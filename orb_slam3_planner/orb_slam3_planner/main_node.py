#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
import time

import sensor_msgs_py.point_cloud2 as pc2
from orb_slam3_planner.map_builder_module import MapBuilder
from orb_slam3_planner.planner_module import FrontierPlanner
from orb_slam3_planner.drone_controller_module import DroneController
from slam_msgs.srv import GetMapStatus


class AutonomousExplorerNode(Node):
    """
    The central node that coordinates mapping, planning, and motion control for autonomous exploration.

    Responsibilities:
    - Subscribe to SLAM pose and point cloud
    - Maintain occupancy grid
    - Publish velocity, map, and visualization messages
    - Manage exploration state machine
    """

    def __init__(self):
        super().__init__('autonomous_explorer_node')

        # ======================
        # Namespace & Parameters
        # ======================
        self.declare_parameter('robot_namespace', '')
        self.robot_namespace = self.get_parameter('robot_namespace').value.rstrip('/')

        # ======================
        # Map Parameters
        # ======================
        self.cell_size = 0.25
        self.map_range = 20.0
        self.grid_size = int(2 * self.map_range / self.cell_size)

        self.occupancy_prob = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)
        self.update_count = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

        # Temporal map for SLAM loss recovery
        self.temporal_occupancy_prob = None
        self.temporal_update_count = None
        self.using_temporal_map = False

        self.height_min = 0.1
        self.height_max = 2.0
        self.obstacle_prob_increment = 0.2
        self.free_prob_decrement = -0.05
        self.occupied_threshold = 0.75
        self.free_threshold = 0.35
        self.freeze_update_count = 8

        # ======================
        # Sensor Parameters
        # ======================
        self.camera_fov = math.radians(60)
        self.camera_range = 10.0
        self.min_points_for_obstacle = 20

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
        # SLAM Tracking Defense
        # ======================
        self.last_pose_time = None
        self.last_valid_pose = None
        self.pose_timeout = 4.0  # seconds without pose = tracking lost
        self.recovery_spin_time = 8.0  # seconds to spin when lost
        self.slam_lost_start_time = None
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3

        # Position jump detection
        self.max_position_jump = 5.0  # meters
        self.last_robot_world_pos = None

        # Point cloud monitoring
        self.recent_point_counts = []
        self.point_count_window = 5
        self.min_point_ratio = 0.3  # Minimum ratio compared to average

        # SLAM correction mode
        self.slam_correction_mode = False
        self.correction_start_time = None
        self.correction_spin_time = 5.0  # seconds to spin for correction
        self.last_correction_pose = None
        self.correction_stable_count = 0
        self.correction_stable_threshold = 5  # iterations without change
        self.position_stability_threshold = 0.1  # meters

        # ======================
        # ROS2 Setup
        # ======================
        ns = f'/{self.robot_namespace}' if self.robot_namespace else ''

        self.cmd_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, f'{ns}/planner_occupancy_grid', 10)
        self.goal_pub = self.create_publisher(Point, f'{ns}/goal_grid_pos', 10)
        self.robot_pos_pub = self.create_publisher(Point, f'{ns}/robot_grid_pos', 10)
        self.direction_pub = self.create_publisher(PoseStamped, f'{ns}/robot_direction', 10)

        self.mapper = MapBuilder(self)
        self.planner = FrontierPlanner(self)
        self.controller = DroneController(self)

        self.create_subscription(PointCloud2, f'{ns}/orb_slam3/landmarks_raw', self.mapper.pointcloud_callback, 10)
        self.create_subscription(PoseStamped, f'{ns}/robot_pose_slam', self.pose_callback, 10)

        self.create_timer(0.5, self.control_loop)
        self.create_timer(1.0, self.check_tracking_lost)  # Check for tracking loss

        self.get_logger().info(f"Autonomous Explorer Node started for namespace: {self.robot_namespace}")

    def update_cell_probability(self, x, y, prob_change):
        """
        Update a cell's occupancy probability using Bayesian-like updates.

        Args:
            x (int): Grid x index
            y (int): Grid y index
            prob_change (float): Positive for obstacles, negative for free space
        """
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return

        # Use temporal map if SLAM is lost
        if self.using_temporal_map:
            old_prob = self.temporal_occupancy_prob[y, x]

            if prob_change > 0:
                new_prob = old_prob + prob_change * (1 - old_prob)
            else:
                new_prob = old_prob + prob_change * old_prob

            self.temporal_occupancy_prob[y, x] = np.clip(new_prob, 0.01, 0.99)
            self.temporal_update_count[y, x] += 1
        else:
            old_prob = self.occupancy_prob[y, x]

            if prob_change > 0:
                new_prob = old_prob + prob_change * (1 - old_prob)
            else:
                new_prob = old_prob + prob_change * old_prob

            self.occupancy_prob[y, x] = np.clip(new_prob, 0.01, 0.99)
            self.update_count[y, x] += 1

    def pose_callback(self, msg):
        """
        Update the robot's position and heading from incoming SLAM pose messages.

        Args:
            msg (geometry_msgs.msg.PoseStamped): The robot's current estimated pose.
        """
        current_time = time.time()

        # Check for position jump
        world_x = msg.pose.position.x
        world_y = msg.pose.position.y

        if self.last_robot_world_pos is not None:
            distance_jumped = math.sqrt(
                (world_x - self.last_robot_world_pos[0]) ** 2 +
                (world_y - self.last_robot_world_pos[1]) ** 2
            )

            if distance_jumped > self.max_position_jump:
                # Check if jump is to origin (complete tracking loss) or elsewhere (localization error)
                distance_from_origin = math.sqrt(world_x ** 2 + world_y ** 2)

                if distance_from_origin < 0.5:  # Jump to near origin indicates tracking loss
                    self.get_logger().error(f"SLAM tracking lost! Jump to origin detected: {distance_jumped:.2f}m")
                    self.enter_slam_lost_state()
                    return
                else:
                    # Jump to non-origin indicates localization error
                    self.get_logger().warn(
                        f"SLAM localization jump detected: {distance_jumped:.2f}m to ({world_x:.2f}, {world_y:.2f})")
                    if not self.slam_correction_mode:
                        self.enter_slam_correction_mode()
                    # Don't update position during correction mode
                    return

        # If in correction mode, check for position stability
        if self.slam_correction_mode:
            if self.last_correction_pose is not None:
                pose_change = math.sqrt(
                    (world_x - self.last_correction_pose[0]) ** 2 +
                    (world_y - self.last_correction_pose[1]) ** 2
                )

                if pose_change < self.position_stability_threshold:
                    self.correction_stable_count += 1
                    if self.correction_stable_count >= self.correction_stable_threshold:
                        self.get_logger().info("SLAM position stabilized, exiting correction mode")
                        self.exit_slam_correction_mode()
                else:
                    self.correction_stable_count = 0
                    self.get_logger().info(f"SLAM still correcting, position changed by {pose_change:.3f}m")

            self.last_correction_pose = (world_x, world_y)

        self.last_robot_world_pos = (world_x, world_y)
        self.last_pose_time = current_time

        # If we were in SLAM_LOST, recover
        if self.state == "SLAM_LOST":
            self.get_logger().info("SLAM tracking recovered!")
            self.exit_slam_lost_state()
            self.state = "EXPLORING"

        self.current_pose = msg.pose
        self.last_valid_pose = msg.pose

        # Convert to grid coordinates
        grid_x = int((world_x + self.map_range) / self.cell_size)
        grid_y = int((world_y + self.map_range) / self.cell_size)

        # Extract heading angle
        q = msg.pose.orientation
        self.robot_angle = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            self.robot_pos = (grid_x, grid_y)

            # Publish robot position (keeping your exact format)
            robot_msg = Point()
            robot_msg.x = float(grid_x)
            robot_msg.y = float(grid_y)
            robot_msg.z = float(self.robot_angle)
            self.robot_pos_pub.publish(robot_msg)

    def check_tracking_lost(self):
        """Check if SLAM tracking is lost"""
        if self.last_pose_time is None:
            return

        time_since_pose = time.time() - self.last_pose_time

        if time_since_pose > self.pose_timeout and self.state != "SLAM_LOST":
            self.get_logger().error(f"SLAM tracking lost! No pose for {time_since_pose:.1f}s")
            self.enter_slam_lost_state()

    def enter_slam_correction_mode(self):
        """Enter SLAM correction mode to help SLAM correct localization error"""
        self.slam_correction_mode = True
        self.correction_start_time = time.time()
        self.last_correction_pose = None
        self.correction_stable_count = 0

        # Store current state to resume later
        self.pre_correction_state = self.state
        self.pre_correction_target = self.target
        self.pre_correction_path = self.current_path
        self.pre_correction_path_index = self.path_index

        self.state = "SLAM_CORRECTION"

        # Immediately stop
        twist = Twist()
        self.cmd_pub.publish(twist)

        self.get_logger().warn("Entered SLAM_CORRECTION mode to fix localization")

    def exit_slam_correction_mode(self):
        """Exit SLAM correction mode and resume previous activity"""
        self.slam_correction_mode = False
        self.correction_start_time = None
        self.last_correction_pose = None
        self.correction_stable_count = 0

        # Restore previous state
        self.state = self.pre_correction_state
        self.target = self.pre_correction_target
        self.current_path = self.pre_correction_path
        self.path_index = self.pre_correction_path_index

        self.get_logger().info("Exited SLAM_CORRECTION mode, resuming exploration")

    def enter_slam_lost_state(self):
        """Enter SLAM lost state and create temporal map"""
        if self.state == "SLAM_LOST":
            return

        self.state = "SLAM_LOST"
        self.slam_lost_start_time = time.time()
        self.recovery_attempts = 0

        # Create temporal map as copy of current map
        self.temporal_occupancy_prob = self.occupancy_prob.copy()
        self.temporal_update_count = self.update_count.copy()
        self.using_temporal_map = True

        # Immediately stop
        twist = Twist()
        self.cmd_pub.publish(twist)

        self.get_logger().warn("Entered SLAM_LOST state, created temporal map")

    def exit_slam_lost_state(self):
        """Exit SLAM lost state and discard temporal map"""
        self.using_temporal_map = False
        self.temporal_occupancy_prob = None
        self.temporal_update_count = None
        self.recovery_attempts = 0
        self.get_logger().info("Exited SLAM_LOST state, discarded temporal map")

    def monitor_point_cloud_count(self, point_count):
        """Monitor point cloud counts to detect sudden drops"""
        self.recent_point_counts.append(point_count)

        if len(self.recent_point_counts) > self.point_count_window:
            self.recent_point_counts.pop(0)

        if len(self.recent_point_counts) == self.point_count_window:
            avg_count = np.mean(self.recent_point_counts[:-1])
            if avg_count > 0 and point_count < avg_count * self.min_point_ratio:
                self.get_logger().error(f"Point cloud drop detected: {point_count} vs avg {avg_count:.0f}")
                self.enter_slam_lost_state()

    def control_loop(self):
        """
        Main control loop: executes exploration state machine and sends movement commands.
        """

        # Handle SLAM correction mode
        if self.state == "SLAM_CORRECTION":
            time_correcting = time.time() - self.correction_start_time

            if time_correcting < self.correction_spin_time:
                # Spin slowly to help SLAM correct its localization
                twist = Twist()
                twist.angular.z = self.angular_speed * 0.3  # Slower than recovery spin
                self.cmd_pub.publish(twist)

                remaining_time = self.correction_spin_time - time_correcting
                self.get_logger().info(f"SLAM correction: spinning slowly ({remaining_time:.1f}s remaining)")
            else:
                # Timeout - accept current position and continue
                self.get_logger().warn("SLAM correction timeout, accepting current position")
                self.exit_slam_correction_mode()
            return

        # Handle SLAM lost state
        if self.state == "SLAM_LOST":
            time_lost = time.time() - self.slam_lost_start_time

            if self.recovery_attempts < self.max_recovery_attempts:
                if time_lost < self.recovery_spin_time:
                    # Spin slowly to find features
                    twist = Twist()
                    twist.angular.z = self.angular_speed * 0.4
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"SLAM lost: spinning to recover ({time_lost:.1f}s)")
                else:
                    # Try backing up and spinning again
                    self.recovery_attempts += 1
                    self.slam_lost_start_time = time.time()

                    # Back up for 2 seconds
                    twist = Twist()
                    twist.linear.x = -self.linear_speed * 0.5
                    self.cmd_pub.publish(twist)
                    # time.sleep(2.0)

                    self.get_logger().info(f"Recovery attempt {self.recovery_attempts}/{self.max_recovery_attempts}")
            else:
                # Stop and wait for manual intervention
                self.controller.stop_robot()
                self.get_logger().error("SLAM recovery failed. Manual restart required.")
            return

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
            # twist.angular.z = -self.angular_speed * 1.5
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

    def publish_map(self):
        """
        Publish the occupancy grid map based on the current probability grid.
        """
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.map_range
        msg.info.origin.position.y = -self.map_range

        # Use temporal map if SLAM is lost
        prob_grid = self.temporal_occupancy_prob if self.using_temporal_map else self.occupancy_prob
        count_grid = self.temporal_update_count if self.using_temporal_map else self.update_count

        ros_grid = []
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                value = self.get_occupancy_value(x, y)
                if value == -1:
                    ros_grid.append(-1)
                elif value == 0:
                    ros_grid.append(0)
                else:
                    ros_grid.append(100)

        msg.data = ros_grid
        self.map_pub.publish(msg)

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

        # Use temporal map if SLAM is lost
        if self.using_temporal_map:
            prob = self.temporal_occupancy_prob[y, x]
            updates = self.temporal_update_count[y, x]
        else:
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