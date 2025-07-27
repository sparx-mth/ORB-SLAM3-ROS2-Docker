#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

from orb_slam3_planner.planner_module import FrontierPlanner
from orb_slam3_planner.drone_controller_module import DroneController


class AutonomousExplorerNode(Node):
    """
    The central node that coordinates planning and motion control for autonomous exploration.
    Updated to work with the efficient occupancy grid mapper.
    """

    def __init__(self, robot_configs):
        super().__init__('autonomous_explorer_node')

        # ======================
        # Namespace & Parameters
        # ======================
        self.declare_parameter('robot_namespace', '')
        self.robot_namespace = self.get_parameter('robot_namespace').value.rstrip('/')

        # Extract robot ID from namespace (assumes format 'robot_0', 'robot_1', etc.)
        self.robot_id = int(self.robot_namespace.split('_')[-1]) if self.robot_namespace else 0

        # Robot configurations
        self.robot_configs = robot_configs
        self.all_robot_ids = list(robot_configs.keys())

        # ======================
        # Map Parameters - Updated to match efficient mapper
        # ======================
        self.resolution = 0.5  # Match efficient mapper's resolution
        self.map_size_meters = 30.0  # Match efficient mapper's map size
        self.grid_size = int(self.map_size_meters / self.resolution)
        self.origin_offset = self.map_size_meters / 2.0

        # For compatibility with existing code
        self.cell_size = self.resolution
        self.map_range = self.origin_offset

        # Local copy of the shared map (-1=unknown, 0=free, 100=occupied)
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # ======================
        # Motion Parameters
        # ======================
        self.linear_speed = 0.4
        self.angular_speed = 0.5
        self.safe_distance = 5

        self.adaptive_speed = True
        self.min_linear_speed = 0.3
        self.max_linear_speed = 0.6

        # ======================
        # Frontier Planning Parameters
        # ======================
        self.use_frontier_scoring = True
        self.visited_targets = set()
        self.exploration_radius = 3  # Increased for larger grid

        # ======================
        # Multi-Robot Coordination
        # ======================
        self.other_robot_positions = {}  # {robot_id: (grid_x, grid_y)}
        self.other_robot_goals = {}  # {robot_id: (grid_x, grid_y)}

        # Weights for multi-robot coordination in scoring
        self.robot_position_weight = 1.5  # Weight for distance from other robots
        self.robot_goal_weight = 2.0  # Weight for distance from other robots' goals
        self.min_robot_separation = 10  # Adjusted for finer resolution

        # ======================
        # Robot State
        # ======================
        self.robot_pos = None
        self.robot_world_pos = None
        self.robot_angle = 0.0
        self.current_pose = None
        self.target = None
        self.state = "EXPLORING"

        self.collision_counter = 0
        self.stuck_counter = 0
        self.last_robot_pos = None

        # A* path following
        self.current_path = []
        self.path_index = 0
        self.waypoint_tolerance = 5  # Adjusted for finer resolution

        # ======================
        # Goal Timeout
        # ======================
        self.target_start_time = None
        self.target_timeout = 30.0  # seconds to reach a goal

        # Transformation matrix for this robot
        config = robot_configs[self.robot_id]
        self.robot_transform = self.create_transformation_matrix(
            config['position'], config.get('orientation', [0, 0, 0])
        )

        # ======================
        # ROS2 Setup
        # ======================
        ns = f'/{self.robot_namespace}' if self.robot_namespace else ''

        self.cmd_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Point, f'{ns}/goal_grid_pos', 10)
        self.robot_pos_pub = self.create_publisher(Point, f'{ns}/robot_grid_pos', 10)

        # Pass self to planner so it can access other robot data
        self.planner = FrontierPlanner(self)
        self.controller = DroneController(self)

        # Subscribe to robot pose directly
        self.create_subscription(
            PoseStamped,
            f'/{self.robot_namespace}/robot_pose_slam',
            self.pose_callback,
            10
        )

        # Subscribe to occupancy grid from efficient mapper
        self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',  # Updated topic name
            self.occupancy_grid_callback,
            10
        )

        # Subscribe to other robots' positions and goals
        for robot_id in self.all_robot_ids:
            if robot_id != self.robot_id:  # Don't subscribe to own topics
                # Robot poses
                self.create_subscription(
                    PoseStamped,
                    f'/robot_{robot_id}/robot_pose_slam',
                    self.create_other_robot_pose_callback(robot_id),
                    10
                )

                # Robot goals from autonomous explorer nodes
                self.create_subscription(
                    Point,
                    f'/robot_{robot_id}/goal_grid_pos',
                    self.create_goal_callback(robot_id),
                    10
                )

        self.create_timer(0.5, self.control_loop)

        self.get_logger().info(
            f"Autonomous Explorer Node started for namespace: {self.robot_namespace} (ID: {self.robot_id})")

    def create_transformation_matrix(self, position, orientation):
        """Create 4x4 transformation matrix"""
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        gx = int((x + self.origin_offset) / self.resolution)
        gy = int((y + self.origin_offset) / self.resolution)
        return gx, gy

    def pose_callback(self, msg):
        """
        Handle robot pose updates with proper transformation.
        """
        # Transform to global frame
        local_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            1.0
        ])
        global_pose = self.robot_transform @ local_pose

        # Store world position
        self.robot_world_pos = (global_pose[0], global_pose[1])

        # Convert to grid
        gx, gy = self.world_to_grid(global_pose[0], global_pose[1])

        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.robot_pos = (gx, gy)

            # Calculate heading
            q = msg.pose.orientation
            local_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # Apply transformation rotation
            rot_matrix = self.robot_transform[:3, :3]
            transform_yaw = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])
            self.robot_angle = local_yaw + transform_yaw

            # Publish grid position for visualization
            grid_msg = Point()
            grid_msg.x = float(gx)
            grid_msg.y = float(gy)
            grid_msg.z = float(self.robot_angle)
            self.robot_pos_pub.publish(grid_msg)

    def create_other_robot_pose_callback(self, robot_id):
        """Create pose callback for other robots"""
        # Get the transformation for this robot
        config = self.robot_configs[robot_id]
        transform = self.create_transformation_matrix(
            config['position'], config.get('orientation', [0, 0, 0])
        )

        def callback(msg):
            # Transform to global frame
            local_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                1.0
            ])
            global_pose = transform @ local_pose

            # Convert to grid
            gx, gy = self.world_to_grid(global_pose[0], global_pose[1])

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                self.other_robot_positions[robot_id] = (gx, gy)

        return callback

    def create_goal_callback(self, robot_id):
        """Create a callback for receiving other robot goals"""

        def callback(msg):
            self.other_robot_goals[robot_id] = (int(msg.x), int(msg.y))

        return callback

    def occupancy_grid_callback(self, msg):
        """
        Update local map from the efficient occupancy grid mapper.
        Simply copy the discrete values: -1=unknown, 0=free, 100=occupied
        """
        if msg.info.width != self.grid_size or msg.info.height != self.grid_size:
            self.get_logger().error(
                f"Map size mismatch: expected {self.grid_size}, got {msg.info.width}x{msg.info.height}")
            return

        # Direct copy of the occupancy grid data
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape((self.grid_size, self.grid_size))

    def control_loop(self):
        """
        Main control loop: executes exploration state machine and sends movement commands.
        """
        if not self.robot_pos:
            self.controller.stop_robot()
            return

        if self.controller.is_stuck():
            self.get_logger().warn("Robot stuck. Switching to RECOVERY.")
            self.state = "RECOVERY"
            self.stuck_counter = 0

        self.last_robot_pos = self.robot_pos

        if self.state == "COLLISION_AVOIDANCE":
            if self.collision_counter > 0:
                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_pub.publish(twist)
                self.collision_counter -= 1
            else:
                self.state = "EXPLORING"

        elif self.state == "RECOVERY":
            twist = Twist()
            twist.linear.x = -self.linear_speed * 1.0
            self.cmd_pub.publish(twist)
            self.state = "EXPLORING"

        elif self.state == "EXPLORING":
            self.target = self.planner.find_nearest_frontier()

            if self.target:
                # Plan path with A*
                path = self.planner.plan_path(self.robot_pos, self.target)
                if path and len(path) > 1:
                    self.current_path = path
                    self.path_index = 1  # Skip current position
                    self.state = "MOVING_TO_TARGET"
                    self.target_start_time = time.time()
                    self.get_logger().info(f"New target: {self.target}, path length: {len(path)}")

                    # Publish the final goal
                    goal_msg = Point()
                    goal_msg.x = float(self.target[0])
                    goal_msg.y = float(self.target[1])
                    goal_msg.z = 0.0
                    self.goal_pub.publish(goal_msg)
                else:
                    self.get_logger().warn(f"No path found to target {self.target}")
                    self.target = None
            else:
                self.controller.turn_to_explore()

        elif self.state == "MOVING_TO_TARGET":
            if not self.target or not self.current_path:
                self.state = "EXPLORING"
                return

            if not self.planner.is_reachable(self.robot_pos[0], self.robot_pos[1]):
                self.state = "EXPLORING"
                return

            # Check timeout
            if self.target_start_time and (time.time() - self.target_start_time) > self.target_timeout:
                self.get_logger().warn(f"Goal timeout! Abandoning target after {self.target_timeout}s")
                self.visited_targets.add((self.target[0], self.target[1]))
                self.target = None
                self.current_path = []
                self.target_start_time = None
                self.state = "EXPLORING"
                return

            rx, ry = self.robot_pos

            # Check if we reached the final target
            tx, ty = self.target
            distance_to_target = math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2)

            if distance_to_target < self.exploration_radius:
                self.get_logger().info("Reached target.")
                self.visited_targets.add((tx, ty))
                self.state = "EXPLORING"
                self.target = None
                self.current_path = []
                self.target_start_time = None
                self.controller.turn_to_explore()
                return

            # Follow the path
            if self.path_index < len(self.current_path):
                waypoint = self.current_path[self.path_index]
                wx, wy = waypoint

                # Check distance to current waypoint
                distance_to_waypoint = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)

                if distance_to_waypoint < self.waypoint_tolerance:
                    # Reached waypoint, move to next
                    self.path_index += 1
                    if self.path_index >= len(self.current_path):
                        # Path completed but haven't reached target, replan
                        self.get_logger().info("Path completed, replanning...")
                        self.state = "EXPLORING"
                        return
                else:
                    # Move toward current waypoint
                    if self.controller.check_collision_ahead():
                        # Obstacle detected, replan path
                        self.get_logger().warn("Obstacle on path! Replanning...")
                        new_path = self.planner.plan_path(self.robot_pos, self.target)
                        if new_path and len(new_path) > 1:
                            self.current_path = new_path
                            self.path_index = 1
                        else:
                            self.get_logger().warn("No alternate path found!")
                            self.state = "EXPLORING"
                            return
                    else:
                        self.controller.move_toward_waypoint(waypoint)
            else:
                # Path index out of bounds, replan
                self.state = "EXPLORING"

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_occupancy_value(self, x, y):
        """
        Get occupancy value for a grid cell.
        Returns: -1=unknown, 0=free, 100=occupied
        """
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return -1

        return self.occupancy_grid[y, x]


def main(args=None):
    rclpy.init(args=args)

    # Robot configurations - must match map merger and mapper
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        1: {'position': [-1.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 2: {'position': [5.0, 5.0, 0.5], 'orientation': [0.0, 0.0, 0.0]}
    }

    node = AutonomousExplorerNode(robot_configs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()