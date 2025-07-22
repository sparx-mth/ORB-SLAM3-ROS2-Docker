#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
import threading


class MultiRobotController(Node):
    """
    Controller for individual robots in the multi-robot system.
    Each robot has its own instance that receives goals and navigates.
    """

    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}_controller')

        self.robot_id = robot_id

        # Motion parameters
        self.linear_speed = 0.4
        self.angular_speed = 0.5
        self.min_linear_speed = 0.3
        self.max_linear_speed = 0.6

        # State
        self.current_goal = None
        self.current_waypoint = None
        self.robot_pose = None
        self.robot_angle = 0.0
        self.grid_position = None

        # Map data
        self.occupancy_grid = None
        self.grid_size = 0
        self.cell_size = 0.25
        self.map_range = 40.0

        # Control parameters
        self.waypoint_tolerance = 3  # cells
        self.goal_tolerance = 3  # cells
        self.obstacle_check_distance = 5  # cells

        # State machine
        self.state = "IDLE"  # IDLE, MOVING_TO_WAYPOINT, AVOIDING_OBSTACLE, EXPLORING
        self.stuck_counter = 0
        self.last_position = None

        # Thread safety
        self.control_lock = threading.Lock()

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, f'/robot_{robot_id}/cmd_vel', 10
        )

        # Subscriptions
        self.create_subscription(
            Point, f'/robot_{robot_id}/goal',
            self.goal_callback, 10
        )

        self.create_subscription(
            Point, f'/robot_{robot_id}/next_waypoint',
            self.waypoint_callback, 10
        )

        self.create_subscription(
            Point, f'/robot_{robot_id}/grid_position',
            self.grid_position_callback, 10
        )

        self.create_subscription(
            PoseStamped, f'/robot_{robot_id}/robot_pose_slam',
            self.pose_callback, 10
        )

        self.create_subscription(
            OccupancyGrid, '/shared_occupancy_grid',
            self.map_callback, 10
        )

        # Control timer
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Robot {robot_id} controller initialized')

    def goal_callback(self, msg):
        """Receive new goal from planner"""
        with self.control_lock:
            self.current_goal = (int(msg.x), int(msg.y))
            self.state = "MOVING_TO_WAYPOINT"
            self.get_logger().info(f'Robot {self.robot_id} received goal: ({int(msg.x)}, {int(msg.y)})')

    def waypoint_callback(self, msg):
        """Receive next waypoint on path"""
        with self.control_lock:
            self.current_waypoint = (int(msg.x), int(msg.y))

    def grid_position_callback(self, msg):
        """Update robot's grid position"""
        with self.control_lock:
            self.grid_position = (int(msg.x), int(msg.y))
            self.robot_angle = msg.z

    def pose_callback(self, msg):
        """Update robot pose"""
        with self.control_lock:
            self.robot_pose = msg.pose

            # Extract angle
            q = msg.pose.orientation
            self.robot_angle = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def map_callback(self, msg):
        """Update occupancy grid"""
        with self.control_lock:
            self.grid_size = msg.info.width
            self.cell_size = msg.info.resolution
            self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def control_loop(self):
        """Main control loop"""
        with self.control_lock:
            # Check if we're stuck
            if self.is_stuck():
                self.state = "AVOIDING_OBSTACLE"
                self.stuck_counter = 0

            # State machine
            if self.state == "IDLE":
                self.stop_robot()

            elif self.state == "MOVING_TO_WAYPOINT":
                if self.current_waypoint and self.grid_position:
                    # Check if we reached the waypoint
                    wx, wy = self.current_waypoint
                    rx, ry = self.grid_position
                    distance = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)

                    if distance < self.waypoint_tolerance:
                        # Check if this was the goal
                        if self.current_goal and self.current_waypoint == self.current_goal:
                            self.get_logger().info(f'Robot {self.robot_id} reached goal!')
                            self.current_goal = None
                            self.current_waypoint = None
                            self.state = "IDLE"
                        else:
                            # Wait for next waypoint
                            self.stop_robot()
                    else:
                        # Check for obstacles
                        if self.check_obstacle_ahead():
                            self.state = "AVOIDING_OBSTACLE"
                        else:
                            self.move_toward_position(self.current_waypoint)
                else:
                    # No waypoint, explore randomly
                    self.state = "EXPLORING"

            elif self.state == "AVOIDING_OBSTACLE":
                # Simple obstacle avoidance - turn away
                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_pub.publish(twist)

                # Check if we can move forward again
                if not self.check_obstacle_ahead():
                    self.state = "MOVING_TO_WAYPOINT"

            elif self.state == "EXPLORING":
                # Random exploration when no goal
                self.explore_randomly()

    def move_toward_position(self, target):
        """Move robot toward a target position"""
        if not self.grid_position:
            return

        rx, ry = self.grid_position
        tx, ty = target

        # Calculate angle to target
        target_angle = math.atan2(ty - ry, tx - rx)
        angle_diff = self.normalize_angle(target_angle - self.robot_angle)

        # Determine speed based on local obstacle density
        speed = self.get_adaptive_speed()

        # Turn toward target if needed
        if abs(angle_diff) > 0.3:
            twist = Twist()
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.cmd_pub.publish(twist)
        else:
            # Move forward with slight turning
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = angle_diff * 0.5
            self.cmd_pub.publish(twist)

    def explore_randomly(self):
        """Random exploration behavior"""
        twist = Twist()

        # Check obstacles
        if self.check_obstacle_ahead():
            # Turn
            twist.angular.z = self.angular_speed
        else:
            # Move forward with slight random turning
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = (np.random.random() - 0.5) * self.angular_speed * 0.3

        self.cmd_pub.publish(twist)

    def check_obstacle_ahead(self):
        """Check for obstacles in front of robot"""
        if not self.grid_position or self.occupancy_grid is None:
            return False

        rx, ry = self.grid_position

        # Check cone in front of robot
        for dist in range(1, self.obstacle_check_distance + 1):
            for offset in range(-1, 2):
                check_angle = self.robot_angle + (offset * 0.2 / dist)
                check_x = int(rx + dist * math.cos(check_angle))
                check_y = int(ry + dist * math.sin(check_angle))

                if 0 <= check_x < self.grid_size and 0 <= check_y < self.grid_size:
                    if self.occupancy_grid[check_y, check_x] == 100:  # Occupied
                        return True

        return False

    def get_adaptive_speed(self):
        """Adjust speed based on obstacle density"""
        if not self.grid_position or self.occupancy_grid is None:
            return self.linear_speed

        rx, ry = self.grid_position
        obstacle_count = 0
        check_radius = 5
        total_cells = 0

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = rx + dx, ry + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    total_cells += 1
                    if self.occupancy_grid[ny, nx] == 100:
                        obstacle_count += 1

        density = obstacle_count / total_cells if total_cells > 0 else 0.0

        # Linear interpolation between min and max speed
        speed_range = self.max_linear_speed - self.min_linear_speed
        adaptive_speed = self.max_linear_speed - (density * speed_range)

        return np.clip(adaptive_speed, self.min_linear_speed, self.max_linear_speed)

    def is_stuck(self):
        """Check if robot is stuck"""
        if not self.grid_position:
            return False

        if self.last_position == self.grid_position:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        self.last_position = self.grid_position
        return self.stuck_counter > 20

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)

    # Get robot ID from command line or environment
    import sys
    if len(sys.argv) > 1:
        robot_id = int(sys.argv[1])
    else:
        robot_id = 0

    node = MultiRobotController(robot_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()