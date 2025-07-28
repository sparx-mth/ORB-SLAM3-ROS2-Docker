#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import math
import numpy as np
import time

from orb_slam3_planner.planner_module import FrontierPlanner
from orb_slam3_planner.drone_controller_module import DroneController


class AutonomousExplorerNode(Node):
    """
    The central node that coordinates planning and motion control for autonomous exploration.
    Adapted to work with the new mapping system.
    """

    def __init__(self):
        super().__init__('autonomous_explorer_node')

        # ======================
        # Namespace & Parameters
        # ======================
        self.declare_parameter('robot_namespace', '')
        self.robot_namespace = self.get_parameter('robot_namespace').value.rstrip('/')

        # Extract robot ID from namespace (assumes format 'robot_0', 'robot_1', etc.)
        self.robot_id = int(self.robot_namespace.split('_')[-1]) if self.robot_namespace else 0

        # List of all robot IDs in the system
        self.all_robot_ids = [0, 1, 2]  # Adjust based on your system

        # ======================
        # Map Parameters (matching new mapper)
        # ======================
        self.cell_size = 0.5  # Resolution from new mapper
        self.map_size_meters = 30.0  # Total map size (30m x 30m)
        self.grid_size = int(self.map_size_meters / self.cell_size)
        self.map_range = self.map_size_meters / 2.0  # For world coordinate conversion

        # Local copy of the shared map
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # ======================
        # Motion Parameters
        # ======================
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.safe_distance = 2

        self.adaptive_speed = False
        self.min_linear_speed = 0.3
        self.max_linear_speed = 0.6

        # ======================
        # Initial 360 Turn Parameters
        # ======================
        self.initial_turn_complete = False
        self.initial_turn_start_angle = None
        self.initial_turn_total_rotation = 0.0
        self.last_angle_for_turn = None
        self.turn_direction = 1  # 1 for counter-clockwise, -1 for clockwise

        # ======================
        # Frontier Planning Parameters
        # ======================
        self.use_frontier_scoring = True
        self.visited_targets = set()
        self.exploration_radius = 1

        # ======================
        # Multi-Robot Coordination
        # ======================
        self.other_robot_positions = {}  # {robot_id: (grid_x, grid_y)}
        self.other_robot_goals = {}  # {robot_id: (grid_x, grid_y)}

        # Weights for multi-robot coordination in scoring
        self.robot_position_weight = 1.5
        self.robot_goal_weight = 2.0
        self.min_robot_separation = 5

        # ======================
        # Robot State
        # ======================
        self.robot_pos = None
        self.robot_angle = 0.0
        self.target = None
        self.state = "INITIAL_TURN"  # Start with initial turn state

        self.collision_counter = 0
        self.stuck_counter = 0
        self.last_robot_pos = None

        # A* path following
        self.current_path = []
        self.path_index = 0
        self.waypoint_tolerance = 2

        # ======================
        # Goal Timeout
        # ======================
        self.target_start_time = None
        self.target_timeout = 40.0  # seconds to reach a goal

        # ======================
        # ROS2 Setup
        # ======================
        ns = f'/{self.robot_namespace}' if self.robot_namespace else ''

        self.cmd_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Point, f'{ns}/goal_grid_pos', 10)
        self.path_pub = self.create_publisher(Path, f'{ns}/planned_path', 10)

        # Pass self to planner so it can access other robot data
        self.planner = FrontierPlanner(self)
        self.controller = DroneController(self)

        # Subscribe to shared occupancy grid
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.map_callback, 10)

        # Subscribe to robot positions from map builder
        self.create_subscription(PoseArray, '/robot_grid_positions',
                                 self.robot_positions_callback, 10)

        # Subscribe to other robots' goals
        for robot_id in self.all_robot_ids:
            if robot_id != self.robot_id:
                self.create_subscription(
                    Point, f'/robot_{robot_id}/goal_grid_pos',
                    self.create_goal_callback(robot_id), 10
                )

        self.create_timer(0.5, self.control_loop)

        self.get_logger().info(
            f"Autonomous Explorer Node started for robot_{self.robot_id} - Starting with 360 turn")

    def create_goal_callback(self, robot_id):
        """Create a callback for receiving other robot goals"""

        def callback(msg):
            self.other_robot_goals[robot_id] = (int(msg.x), int(msg.y))

        return callback

    def robot_positions_callback(self, msg):
        """
        Handle robot positions from the map builder.
        The PoseArray contains positions for all robots in grid coordinates.
        Robot ID is encoded in the z position.
        """
        for pose in msg.poses:
            robot_id = int(pose.position.z)
            gx = int(pose.position.x)
            gy = int(pose.position.y)

            # Extract heading from quaternion
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            if robot_id == self.robot_id:
                # Update own position
                self.robot_pos = (gx, gy)
                self.robot_angle = yaw
            else:
                # Update other robot positions
                self.other_robot_positions[robot_id] = (gx, gy)

    def map_callback(self, msg):
        """
        Update local map from the shared occupancy grid.
        """
        if msg.info.width != self.grid_size or msg.info.height != self.grid_size:
            # Update grid size if map size changed
            self.grid_size = msg.info.width
            self.cell_size = msg.info.resolution
            self.map_size_meters = self.grid_size * self.cell_size
            self.map_range = self.map_size_meters / 2.0
            self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Convert ROS occupancy grid to internal format
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                idx = y * self.grid_size + x
                value = msg.data[idx]
                self.occupancy_grid[y, x] = value

    def publish_goal(self):
        """Publish current goal for visualization"""
        if self.target:
            goal_msg = Point()
            goal_msg.x = float(self.target[0])
            goal_msg.y = float(self.target[1])
            goal_msg.z = 0.0
            self.goal_pub.publish(goal_msg)

    def publish_path(self):
        """Publish current path for visualization"""
        if self.current_path and len(self.current_path) > 0:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"

            for gx, gy in self.current_path:
                pose = PoseStamped()
                pose.header = path_msg.header
                # Convert grid to world coordinates for Path message
                pose.pose.position.x = gx * self.cell_size - self.map_range
                pose.pose.position.y = gy * self.cell_size - self.map_range
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)

    def perform_initial_turn(self):
        """
        Perform a 360-degree turn to scan the environment before starting exploration.
        Returns True when the turn is complete.
        """
        if self.robot_angle is None:
            return False

        # Initialize turn tracking
        if self.initial_turn_start_angle is None:
            self.initial_turn_start_angle = self.robot_angle
            self.last_angle_for_turn = self.robot_angle
            self.initial_turn_total_rotation = 0.0
            self.get_logger().info(f"Starting 360 turn from angle: {math.degrees(self.robot_angle):.1f}°")

        # Calculate angle change since last update
        angle_diff = self.robot_angle - self.last_angle_for_turn

        # Handle angle wrap-around
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Accumulate total rotation
        self.initial_turn_total_rotation += abs(angle_diff)
        self.last_angle_for_turn = self.robot_angle

        # Check if we've completed a full rotation (with some tolerance)
        if self.initial_turn_total_rotation >= 2 * math.pi - 0.1:
            self.get_logger().info(
                f"Completed 360 turn! Total rotation: {math.degrees(self.initial_turn_total_rotation):.1f}°")
            return True

        # Continue turning
        twist = Twist()
        twist.angular.z = self.angular_speed * self.turn_direction
        self.cmd_pub.publish(twist)

        # Log progress periodically
        if int(math.degrees(self.initial_turn_total_rotation)) % 45 == 0:
            self.get_logger().info(f"Turn progress: {math.degrees(self.initial_turn_total_rotation):.1f}°")

        return False

    def control_loop(self):
        """
        Main control loop: executes exploration state machine and sends movement commands.
        """
        if not self.robot_pos:
            self.controller.stop_robot()
            return

        # Handle initial 360 turn
        if self.state == "INITIAL_TURN":
            if self.perform_initial_turn():
                self.initial_turn_complete = True
                self.state = "EXPLORING"
                self.controller.stop_robot()  # Brief stop before starting exploration
                self.get_logger().info("Initial 360 turn complete. Starting exploration.")
            return

        # Check if robot is stuck (only after initial turn)
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

                    # Publish goal and path for visualization
                    self.publish_goal()
                    self.publish_path()
                else:
                    self.get_logger().warn(f"No path found to target {self.target}")
                    self.target = None
            else:
                self.controller.turn_to_explore()

        elif self.state == "MOVING_TO_TARGET":
            if not self.target or not self.current_path:
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
                            self.publish_path()
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
        Get occupancy value from grid.
        Returns: 100 = occupied, 0 = free, -1 = unknown
        """
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return -1
        return self.occupancy_grid[y, x]


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()