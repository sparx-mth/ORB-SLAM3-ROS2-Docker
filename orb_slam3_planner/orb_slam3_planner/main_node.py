#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
import time

from orb_slam3_planner.planner_module import FrontierPlanner
from orb_slam3_planner.drone_controller_module import DroneController


class AutonomousExplorerNode(Node):
    """
    The central node that coordinates planning and motion control for autonomous exploration.
    In multi-robot mode, it subscribes to the shared occupancy grid instead of building its own map.
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
        # Map Parameters
        # ======================
        self.cell_size = 0.25
        self.map_range = 15.0
        self.grid_size = int(2 * self.map_range / self.cell_size)

        # Local copy of the shared map
        self.occupancy_prob = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)
        self.update_count = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

        self.occupied_threshold = 0.75
        self.free_threshold = 0.35

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
        self.exploration_radius = 2

        # ======================
        # Multi-Robot Coordination
        # ======================
        self.other_robot_positions = {}  # {robot_id: (grid_x, grid_y)}
        self.other_robot_goals = {}  # {robot_id: (grid_x, grid_y)}

        # Weights for multi-robot coordination in scoring
        self.robot_position_weight = 1.5  # Weight for distance from other robots
        self.robot_goal_weight = 2.0  # Weight for distance from other robots' goals
        self.min_robot_separation = 5  # Minimum desired grid cells between robots

        # ======================
        # Robot State
        # ======================
        self.robot_pos = None
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
        self.waypoint_tolerance = 2  # cells

        # ======================
        # Goal Timeout
        # ======================
        self.target_start_time = None
        self.target_timeout = 30.0  # seconds to reach a goal

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

        # Subscribe to robot pose - this is already in global frame from multi_robot_map_builder
        self.create_subscription(Point, f'/{self.robot_namespace}/grid_position',
                                 self.grid_position_callback, 10)

        # Subscribe to shared occupancy grid instead of building our own
        self.create_subscription(OccupancyGrid, '/shared_occupancy_grid', self.shared_map_callback, 10)

        # Subscribe to other robots' positions and goals
        for robot_id in self.all_robot_ids:
            if robot_id != self.robot_id:  # Don't subscribe to own topics
                # Robot positions from multi_robot_map_builder
                self.create_subscription(
                    Point, f'/robot_{robot_id}/grid_position',
                    self.create_position_callback(robot_id), 10
                )

                # Robot goals from autonomous explorer nodes
                self.create_subscription(
                    Point, f'/robot_{robot_id}/goal_grid_pos',
                    self.create_goal_callback(robot_id), 10
                )

        self.create_timer(0.5, self.control_loop)

        self.get_logger().info(
            f"Autonomous Explorer Node started for namespace: {self.robot_namespace} (ID: {self.robot_id})")

    def create_position_callback(self, robot_id):
        """Create a callback for receiving other robot positions"""

        def callback(msg):
            self.other_robot_positions[robot_id] = (int(msg.x), int(msg.y))

        return callback

    def create_goal_callback(self, robot_id):
        """Create a callback for receiving other robot goals"""

        def callback(msg):
            self.other_robot_goals[robot_id] = (int(msg.x), int(msg.y))

        return callback

    def grid_position_callback(self, msg):
        """
        Receive grid position directly from multi_robot_map_builder.

        Args:
            msg (geometry_msgs.msg.Point): Grid position (x, y) and angle (z)
        """
        grid_x = int(msg.x)
        grid_y = int(msg.y)
        self.robot_angle = msg.z

        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            self.robot_pos = (grid_x, grid_y)
            # Re-publish for visualization
            self.robot_pos_pub.publish(msg)

    def shared_map_callback(self, msg):
        """
        Update local map from the shared occupancy grid.

        Args:
            msg (nav_msgs.msg.OccupancyGrid): The shared occupancy grid
        """
        if msg.info.width != self.grid_size or msg.info.height != self.grid_size:
            self.get_logger().error(
                f"Map size mismatch: expected {self.grid_size}, got {msg.info.width}x{msg.info.height}")
            return

        # Convert ROS occupancy grid to internal probability format
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                idx = y * self.grid_size + x
                value = msg.data[idx]

                if value == -1:  # Unknown
                    self.occupancy_prob[y, x] = 0.5
                    self.update_count[y, x] = 0
                elif value == 0:  # Free
                    self.occupancy_prob[y, x] = 0.2
                    self.update_count[y, x] = 3
                else:  # Occupied (value == 100)
                    self.occupancy_prob[y, x] = 0.9
                    self.update_count[y, x] = 3

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

        Args:
            angle (float): Angle in radians.

        Returns:
            float: Normalized angle.
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_occupancy_value(self, x, y):
        """
        Convert a cell's probability to a discrete occupancy value.

        Args:
            x (int): Grid x coordinate.
            y (int): Grid y coordinate.

        Returns:
            int: 1 = occupied, 0 = free, -1 = unknown
        """
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return -1

        prob = self.occupancy_prob[y, x]
        updates = self.update_count[y, x]

        if updates < 2:
            return -1
        if prob > self.occupied_threshold:
            return 1
        if prob < self.free_threshold:
            return 0
        return -1


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