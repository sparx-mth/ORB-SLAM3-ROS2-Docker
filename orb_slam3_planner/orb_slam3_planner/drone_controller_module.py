import math
import numpy as np
from geometry_msgs.msg import Twist
import time


class DroneController:
    """
    DroneController is responsible for commanding the robot's movement,
    handling obstacle avoidance, stuck recovery, and adaptive speed control.

    Updated to work with discrete occupancy values: -1=unknown, 0=free, 100=occupied
    """

    def __init__(self, node):
        """
        Initialize the DroneController with references to the shared main node.

        Args:
            node (rclpy.node.Node): The central ROS2 node containing shared state and publishers.
        """
        self.node = node
        self.get_logger = node.get_logger
        self.get_clock = node.get_clock
        self.cmd_pub = node.cmd_pub  # Publisher to /cmd_vel

        # Track motion for pausing
        self.last_motion_type = None  # 'turn' or 'move'
        self.motion_completed = False

        self.get_logger().info("DroneController initialized")

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid coordinates to world coordinates.

        Args:
            grid_x, grid_y: Grid coordinates

        Returns:
            world_x, world_y: World coordinates
        """
        world_x = grid_x * self.node.cell_size - self.node.map_range
        world_y = grid_y * self.node.cell_size - self.node.map_range
        return world_x, world_y

    def move_toward_waypoint(self, waypoint):
        """
        Move the robot towards a specific waypoint on the path.

        Args:
            waypoint (tuple): (x, y) grid coordinates of the waypoint
        """
        if not self.node.robot_pos:
            return

        rx, ry = self.node.robot_pos
        wx, wy = waypoint

        # Convert grid coordinates to world coordinates for angle calculation
        robot_world_x, robot_world_y = self.grid_to_world(rx, ry)
        waypoint_world_x, waypoint_world_y = self.grid_to_world(wx, wy)

        # Calculate angle in world coordinates
        target_angle = math.atan2(waypoint_world_y - robot_world_y,
                                  waypoint_world_x - robot_world_x)
        angle_diff = self.node.normalize_angle(target_angle - self.node.robot_angle)

        speed = self.get_adaptive_speed()

        # First align with the target
        if abs(angle_diff) > 0.3:
            twist = Twist()
            twist.angular.z = self.node.angular_speed if angle_diff > 0 else -self.node.angular_speed
            self.cmd_pub.publish(twist)

            # Mark that we just completed a turn
            if self.last_motion_type != 'turn':
                self.last_motion_type = 'turn'
                self.node.last_motion_time = time.time()
                self.node.is_paused = True
            return

        # Then move forward
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angle_diff * 0.5  # Proportional control for small corrections
        self.cmd_pub.publish(twist)

        # Track continuous movement (pause happens at waypoints in main loop)
        self.last_motion_type = 'move'

    def stop_robot(self):
        """
        Immediately stop all robot motion by publishing zero velocities.
        """
        twist = Twist()
        self.cmd_pub.publish(twist)

    def turn_to_explore(self):
        """
        Rotate in place or gently move forward to facilitate exploration when no frontiers are available.
        """
        twist = Twist()
        turn_direction = 1 if int(self.get_clock().now().nanoseconds / 1e9) % 10 < 5 else -1
        twist.angular.z = self.node.angular_speed * 0.5 * turn_direction
        twist.linear.x = self.node.linear_speed * 0.3
        self.cmd_pub.publish(twist)

    def is_stuck(self):
        """
        Check whether the robot is stuck by comparing current position with the last known position.

        Returns:
            bool: True if the robot is not making progress; otherwise, False.
        """
        if not self.node.robot_pos or not self.node.last_robot_pos:
            return False

        if self.node.robot_pos == self.node.last_robot_pos:
            self.node.stuck_counter += 1
        else:
            self.node.stuck_counter = 0

        return self.node.stuck_counter > 20

    def check_collision_ahead(self):
        """
        Look ahead of the robot's current heading for obstacles using the occupancy grid.
        A conical detection area is used to check for potential collisions.

        Returns:
            bool: True if an obstacle is detected ahead; False otherwise.
        """
        if not self.node.robot_pos:
            return False

        rx, ry = self.node.robot_pos
        robot_width_cells = 1
        check_distance = self.node.safe_distance

        grid = self.node.occupancy_grid

        for dist in range(1, check_distance + 1):
            width_at_dist = max(1, robot_width_cells - dist // 3)

            for offset in range(-width_at_dist, width_at_dist + 1):
                check_angle = self.node.robot_angle + (offset * 0.2 / dist)

                # Check in grid coordinates but using the correct angle
                check_x = int(rx + dist * math.cos(check_angle))
                check_y = int(ry + dist * math.sin(check_angle))

                if 0 <= check_x < self.node.grid_size and 0 <= check_y < self.node.grid_size:
                    if grid[check_y, check_x] == 100:  # Direct check for occupied
                        return True

        return False

    def calculate_obstacle_density(self):
        """
        Calculate the local density of obstacles around the robot within a fixed radius.

        Returns:
            float: Ratio of occupied cells to total cells within the check area.
        """
        if not self.node.robot_pos:
            return 0.0

        rx, ry = self.node.robot_pos
        obstacle_count = 0
        check_radius = 5
        total_cells = 0

        grid = self.node.occupancy_grid

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = rx + dx, ry + dy
                if 0 <= nx < self.node.grid_size and 0 <= ny < self.node.grid_size:
                    total_cells += 1
                    if grid[ny, nx] == 100:  # Direct check for occupied
                        obstacle_count += 1

        return obstacle_count / total_cells if total_cells > 0 else 0.0

    def get_adaptive_speed(self):
        """
        Compute the robot's linear speed dynamically based on local obstacle density.

        Returns:
            float: The selected linear speed (m/s) within allowed speed bounds.
        """
        if not self.node.adaptive_speed:
            return self.node.linear_speed

        density = self.calculate_obstacle_density()
        speed_range = self.node.max_linear_speed - self.node.min_linear_speed
        adaptive_speed = self.node.max_linear_speed - (density * speed_range)

        return np.clip(adaptive_speed, self.node.min_linear_speed, self.node.max_linear_speed)