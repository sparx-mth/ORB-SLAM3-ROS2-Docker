import math
import numpy as np
from geometry_msgs.msg import Twist


class DroneController:
    """
    A controller class for managing robot movement in a grid-based exploration system.

    This class interfaces with the main exploration node and issues `Twist` velocity
    commands based on local navigation logic. It supports reactive control for path
    following, obstacle avoidance, recovery from being stuck, and adaptive speed adjustment.

    Key Features:
    -------------
    - Converts grid cell targets to world-space motion.
    - Rotates and drives toward waypoints.
    - Detects obstacles and stops or turns accordingly.
    - Monitors robot progress and triggers recovery when stuck.
    - Dynamically adjusts speed based on local congestion (obstacle density).

    Usage:
    ------
    This controller is initialized with a reference to the main node (which must
    provide pose, map access, and publishers). It is designed to be called from
    within the main control loop during autonomous exploration.

    Expected node attributes:
    -------------------------
    - self.node.robot_pos: Tuple[int, int] – current grid position
    - self.node.robot_angle: float – orientation in radians
    - self.node.grid_size: int – size of occupancy grid
    - self.node.get_occupancy_value(x, y): -> int – access to occupancy map
    - self.node.cmd_pub: ROS2 publisher for geometry_msgs/Twist
    - self.node.linear_speed, angular_speed: default movement speeds
    - self.node.adaptive_speed: bool – enable/disable adaptive speed
    - self.node.min_linear_speed, max_linear_speed: float – speed limits
    - self.node.safe_distance: int – lookahead cells for collision check
    - self.node.stuck_counter: int – counter for tracking immobility
    - self.node.last_robot_pos: Tuple[int, int] – previous robot position
    """

    def __init__(self, node):
        """
        Initialize the controller using shared references from the main node.

        Args:
            node (AutonomousExplorerNode): The main exploration node containing
                                           pose, map, and publishers.
        """
        self.node = node
        self.get_logger = node.get_logger
        self.get_clock = node.get_clock
        self.cmd_pub = node.cmd_pub
        self.get_logger().info("DroneController initialized")

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid cell coordinates to world coordinates (meters).
        """
        world_x = grid_x * self.node.cell_size - self.node.map_range
        world_y = grid_y * self.node.cell_size - self.node.map_range
        return world_x, world_y

    def move_toward_waypoint(self, waypoint):
        """
        Drive the robot toward the given grid waypoint using heading alignment
        followed by forward movement.

        Args:
            waypoint (tuple): Target cell in (grid_x, grid_y).
        """
        if not self.node.robot_pos:
            return

        rx, ry = self.node.robot_pos
        wx, wy = waypoint

        # Convert current and target positions to world space
        robot_world_x, robot_world_y = self.grid_to_world(rx, ry)
        waypoint_world_x, waypoint_world_y = self.grid_to_world(wx, wy)

        # Compute desired heading
        target_angle = math.atan2(waypoint_world_y - robot_world_y,
                                  waypoint_world_x - robot_world_x)
        angle_diff = self.node.normalize_angle(target_angle - self.node.robot_angle)

        speed = self.get_adaptive_speed()

        # Step 1: Rotate toward waypoint
        if abs(angle_diff) > 0.3:
            twist = Twist()
            twist.angular.z = self.node.angular_speed if angle_diff > 0 else -self.node.angular_speed
            self.cmd_pub.publish(twist)
            return

        # Step 2: Drive forward with small angular correction
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angle_diff * 0.5  # Smooth heading correction
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """
        Immediately stop all motion by publishing a zero Twist command.
        """
        twist = Twist()
        self.cmd_pub.publish(twist)

    def turn_to_explore(self):
        """
        Turn in place (with slight forward motion) to scan for frontiers.

        Used when no goals are available.
        """
        twist = Twist()

        # Alternate turn direction every 5 seconds for more coverage
        turn_direction = 1 if int(self.get_clock().now().nanoseconds / 1e9) % 10 < 5 else -1

        twist.angular.z = self.node.angular_speed * 0.5 * turn_direction
        twist.linear.x = self.node.linear_speed * 0.3  # gentle forward drift
        self.cmd_pub.publish(twist)

    def is_stuck(self):
        """
        Determine if the robot is stuck (i.e., not moving for many cycles).

        Returns:
            bool: True if robot hasn't changed position in over 30 cycles.
        """
        if not self.node.robot_pos or not self.node.last_robot_pos:
            return False

        if self.node.robot_pos == self.node.last_robot_pos:
            self.node.stuck_counter += 1
        else:
            self.node.stuck_counter = 0

        return self.node.stuck_counter > 30

    def check_collision_ahead(self):
        """
        Scan a short distance ahead in the current heading to detect obstacles.

        Returns:
            bool: True if an obstacle (value 100) is detected in front.
        """
        if not self.node.robot_pos:
            return False

        rx, ry = self.node.robot_pos
        robot_width_cells = 1  # Approximate width in grid cells
        check_distance = self.node.safe_distance

        for dist in range(1, check_distance + 1):
            # Narrow the beam width at greater distances
            width_at_dist = max(1, robot_width_cells - dist // 3)

            for offset in range(-width_at_dist, width_at_dist + 1):
                check_angle = self.node.robot_angle + (offset * 0.2 / dist)

                # Check in grid coordinates
                check_x = int(rx + dist * math.cos(check_angle))
                check_y = int(ry + dist * math.sin(check_angle))

                if 0 <= check_x < self.node.grid_size and 0 <= check_y < self.node.grid_size:
                    if self.node.get_occupancy_value(check_x, check_y) == 100:  # Occupied
                        return True

        return False

    def calculate_obstacle_density(self):
        """
        Measure how many nearby cells are occupied to assess congestion.

        Returns:
            float: Ratio of occupied cells in a 5x5 radius window.
        """
        if not self.node.robot_pos:
            return 0.0

        rx, ry = self.node.robot_pos
        obstacle_count = 0
        check_radius = 5
        total_cells = 0

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = rx + dx, ry + dy
                if 0 <= nx < self.node.grid_size and 0 <= ny < self.node.grid_size:
                    total_cells += 1
                    if self.node.get_occupancy_value(nx, ny) == 100:
                        obstacle_count += 1

        return obstacle_count / total_cells if total_cells > 0 else 0.0

    def get_adaptive_speed(self):
        """
        Adjust the robot's speed based on local obstacle density.

        Returns:
            float: Linear speed in m/s, clipped between min and max speeds.
        """
        if not self.node.adaptive_speed:
            return self.node.linear_speed

        density = self.calculate_obstacle_density()
        speed_range = self.node.max_linear_speed - self.node.min_linear_speed
        adaptive_speed = self.node.max_linear_speed - (density * speed_range)

        return np.clip(adaptive_speed, self.node.min_linear_speed, self.node.max_linear_speed)
