import math
import numpy as np
from geometry_msgs.msg import Twist


class DroneController:
    """
    DroneController is responsible for commanding the robot's movement,
    handling obstacle avoidance, stuck recovery, and adaptive speed control.
    """

    def __init__(self, node):
        """
        Initialize the DroneController with references to the shared main node.
        """
        self.node = node
        self.get_logger = node.get_logger
        self.get_clock = node.get_clock
        self.cmd_pub = node.cmd_pub
        self.get_logger().info("DroneController initialized")

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid coordinates to world coordinates.
        """
        world_x = grid_x * self.node.cell_size - self.node.map_range
        world_y = grid_y * self.node.cell_size - self.node.map_range
        return world_x, world_y

    def move_toward_waypoint(self, waypoint):
        """
        Move the robot towards a specific waypoint on the path.
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
            return

        # Then move forward
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angle_diff * 0.5  # Proportional control for small corrections
        self.cmd_pub.publish(twist)

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
        Look ahead of the robot's current heading for obstacles using the occupancy grid.
        """
        if not self.node.robot_pos:
            return False

        rx, ry = self.node.robot_pos
        robot_width_cells = 1
        check_distance = self.node.safe_distance

        for dist in range(1, check_distance + 1):
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
        Calculate the local density of obstacles around the robot within a fixed radius.
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
                    if self.node.get_occupancy_value(nx, ny) == 100:  # Occupied
                        obstacle_count += 1

        return obstacle_count / total_cells if total_cells > 0 else 0.0

    def get_adaptive_speed(self):
        """
        Compute the robot's linear speed dynamically based on local obstacle density.
        """
        if not self.node.adaptive_speed:
            return self.node.linear_speed

        density = self.calculate_obstacle_density()
        speed_range = self.node.max_linear_speed - self.node.min_linear_speed
        adaptive_speed = self.node.max_linear_speed - (density * speed_range)

        return np.clip(adaptive_speed, self.node.min_linear_speed, self.node.max_linear_speed)