#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import math
from collections import deque
import time


class MultiRobot2DVisualizer(Node):
    """
    2D visualization of the multi-robot exploration system.
    Now receives robot positions from the map builder instead of SLAM.
    """

    def __init__(self, robot_configs):
        super().__init__('multi_robot_2d_visualizer')

        self.robot_configs = robot_configs
        self.robot_ids = list(robot_configs.keys())

        # Configuration flags
        self.enable_visualization = True  # Master switch for visualization
        self.enable_fov_visualization = True  # Enable FOV cone drawing
        self.enable_ray_tracing = False  # Disable ray tracing visualization for performance
        self.enable_trajectories = True  # Enable trajectory tracking
        self.enable_logging = True  # Enable logging

        # Visualization parameters
        self.scale = 20  # Scale factor for display (reduced for larger map)
        self.window_name = 'Multi-Robot Exploration Map'

        # Robot colors
        self.robot_colors = {
            0: (255, 0, 0),  # Blue
            1: (0, 255, 0),  # Green
            2: (0, 0, 255),  # Red
        }

        # Robot states (in world coordinates)
        self.robot_world_positions = {rid: None for rid in self.robot_ids}
        self.robot_grid_positions = {rid: None for rid in self.robot_ids}
        self.robot_angles = {rid: 0.0 for rid in self.robot_ids}
        self.robot_goals = {rid: None for rid in self.robot_ids}

        # Map data
        self.occupancy_grid = None
        self.grid_size = 0
        self.resolution = 0.5  # Default, will be updated from map
        self.origin_x = 0
        self.origin_y = 0

        # Performance tracking
        self.last_update_time = time.time()
        self.frame_count = 0

        # Trajectories
        if self.enable_trajectories:
            self.trajectories = {rid: deque(maxlen=500) for rid in self.robot_ids}

        # Subscriptions
        # Subscribe to the new occupancy grid topic
        self.create_subscription(
            OccupancyGrid, '/occupancy_grid',
            self.map_callback, 10
        )

        # NEW: Subscribe to robot positions from map builder instead of individual poses
        self.create_subscription(
            PoseArray, '/robot_grid_positions',
            self.robot_positions_callback, 10
        )

        # Robot goals from autonomous explorer nodes (if available)
        for robot_id in self.robot_ids:
            self.create_subscription(
                Point, f'/robot_{robot_id}/goal_grid_pos',
                self.create_goal_callback(robot_id), 10
            )

        if self.enable_logging:
            self.get_logger().info(f'Multi-Robot 2D Visualizer started for robots: {self.robot_ids}')

    def robot_positions_callback(self, msg):
        """
        NEW: Handle robot positions from the map builder.
        The PoseArray contains positions for all robots in grid coordinates.
        Robot ID is encoded in the z position.
        """
        for pose in msg.poses:
            robot_id = int(pose.position.z)
            if robot_id in self.robot_ids:
                gx = int(pose.position.x)
                gy = int(pose.position.y)

                # Extract heading from quaternion
                q = pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

                # Update robot position
                self.robot_grid_positions[robot_id] = (gx, gy)
                self.robot_angles[robot_id] = yaw

                # Calculate world position for display
                world_x = gx * self.resolution - self.resolution * self.grid_size / 2.0
                world_y = gy * self.resolution - self.resolution * self.grid_size / 2.0
                self.robot_world_positions[robot_id] = (world_x, world_y)

                # Track trajectory
                if self.enable_trajectories:
                    self.trajectories[robot_id].append((gx, gy))

    def create_goal_callback(self, robot_id):
        """Factory for goal callbacks"""

        def callback(msg):
            # Store goal position
            self.robot_goals[robot_id] = (int(msg.x), int(msg.y))
            if self.enable_logging:
                self.get_logger().info(
                    f'Robot_{robot_id} goal set to ({int(msg.x)}, {int(msg.y)})'
                )

        return callback

    def map_callback(self, msg):
        """Visualize the occupancy grid with robots"""
        # Skip if visualization is disabled
        if not self.enable_visualization:
            return

        # Update map parameters
        width = msg.info.width
        height = msg.info.height
        self.grid_size = width
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Convert occupancy grid to numpy array
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.occupancy_grid = grid_data

        # Create colored image
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Color coding for map
        img[grid_data == -1] = (128, 128, 128)  # Unknown = Gray
        img[grid_data == 0] = (240, 240, 240)  # Free = Light gray (better visibility)
        img[grid_data == 100] = (50, 50, 50)  # Obstacle = Dark gray

        # Flip image (ROS coordinate system)
        img = cv2.flip(img, 0)

        # Scale up for better visibility
        img_large = cv2.resize(img, (width * self.scale, height * self.scale),
                               interpolation=cv2.INTER_NEAREST)

        # Draw grid lines for better visualization (optional)
        if self.scale >= 4:
            for i in range(0, width * self.scale, self.scale):
                cv2.line(img_large, (i, 0), (i, height * self.scale), (200, 200, 200), 1)
            for i in range(0, height * self.scale, self.scale):
                cv2.line(img_large, (0, i), (width * self.scale, i), (200, 200, 200), 1)

        # Draw trajectories if enabled
        if self.enable_trajectories:
            for robot_id in self.robot_ids:
                if robot_id in self.trajectories and len(self.trajectories[robot_id]) > 1:
                    self.draw_trajectory(img_large, robot_id, height)

        # Draw FOV if enabled
        if self.enable_fov_visualization:
            overlay = img_large.copy()
            for robot_id in self.robot_ids:
                if self.robot_grid_positions[robot_id]:
                    self.draw_robot_fov(overlay, robot_id, height)
            cv2.addWeighted(overlay, 0.3, img_large, 0.7, 0, img_large)

        # Draw goals
        for robot_id in self.robot_ids:
            if self.robot_goals[robot_id]:
                gx, gy = self.robot_goals[robot_id]
                if 0 <= gx < width and 0 <= gy < height:
                    gx_large = gx * self.scale + self.scale // 2
                    gy_large = (height - 1 - gy) * self.scale + self.scale // 2

                    # Draw goal as square with robot's color
                    color = self.robot_colors[robot_id]
                    cv2.rectangle(img_large,
                                  (gx_large - self.scale, gy_large - self.scale),
                                  (gx_large + self.scale, gy_large + self.scale),
                                  color, 2)

                    # Add text label
                    cv2.putText(img_large, f'G{robot_id}',
                                (gx_large + self.scale + 2, gy_large - self.scale - 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw robots
        for robot_id in self.robot_ids:
            if self.robot_grid_positions[robot_id]:
                rx, ry = self.robot_grid_positions[robot_id]
                if 0 <= rx < width and 0 <= ry < height:
                    rx_large = rx * self.scale + self.scale // 2
                    ry_large = (height - 1 - ry) * self.scale + self.scale // 2

                    # Draw robot as circle
                    color = self.robot_colors[robot_id]
                    cv2.circle(img_large, (rx_large, ry_large),
                               radius=max(4, self.scale // 2), color=color, thickness=-1)

                    # Draw black outline
                    cv2.circle(img_large, (rx_large, ry_large),
                               radius=max(4, self.scale // 2) + 1, color=(0, 0, 0), thickness=2)

                    # Draw direction arrow
                    if self.robot_angles[robot_id] is not None:
                        arrow_len = self.scale * 2
                        dx = int(arrow_len * math.cos(self.robot_angles[robot_id]))
                        dy = int(-arrow_len * math.sin(self.robot_angles[robot_id]))

                        cv2.arrowedLine(img_large, (rx_large, ry_large),
                                        (rx_large + dx, ry_large + dy),
                                        (255, 255, 255), 2, tipLength=0.4)

                    # Draw robot ID
                    cv2.putText(img_large, f'R{robot_id}',
                                (rx_large + self.scale, ry_large - self.scale),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(img_large, f'R{robot_id}',
                                (rx_large + self.scale, ry_large - self.scale),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

        # Add info panel with background
        info_height = 20 + len(self.robot_ids) * 25 + 60
        cv2.rectangle(img_large, (0, 0), (300, info_height), (0, 0, 0), -1)
        cv2.rectangle(img_large, (0, 0), (300, info_height), (255, 255, 255), 2)

        cv2.putText(img_large, 'Multi-Robot SLAM Explorer',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(img_large, 'Light=Free, Dark=Obstacle, Gray=Unknown',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Show robot positions
        y_offset = 75
        for robot_id in self.robot_ids:
            if self.robot_world_positions[robot_id]:
                wx, wy = self.robot_world_positions[robot_id]
                color = self.robot_colors[robot_id]
                cv2.putText(img_large, f'R{robot_id}: ({wx:.1f}, {wy:.1f})m',
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                y_offset += 25

        # Add map statistics
        cv2.putText(img_large, f'Resolution: {self.resolution:.2f}m/cell',
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Display
        cv2.imshow(self.window_name, img_large)
        cv2.waitKey(1)

        # FPS tracking
        self.frame_count += 1
        current_time = time.time()
        if self.enable_logging and current_time - self.last_update_time > 5.0:
            fps = self.frame_count / (current_time - self.last_update_time)
            # Count map cells
            unknown = np.sum(grid_data == -1)
            free = np.sum(grid_data == 0)
            occupied = np.sum(grid_data == 100)
            total = width * height

            self.get_logger().info(
                f'Vis FPS: {fps:.1f} | Map: {unknown}/{total} unknown, '
                f'{free}/{total} free, {occupied}/{total} occupied'
            )
            self.last_update_time = current_time
            self.frame_count = 0

    def draw_trajectory(self, img, robot_id, grid_height):
        """Draw robot trajectory if enabled"""
        color = self.robot_colors[robot_id]
        trajectory_points = list(self.trajectories[robot_id])

        for i in range(len(trajectory_points) - 1):
            x1, y1 = trajectory_points[i]
            x2, y2 = trajectory_points[i + 1]

            # Convert to display coordinates
            x1_large = x1 * self.scale + self.scale // 2
            y1_large = (grid_height - 1 - y1) * self.scale + self.scale // 2
            x2_large = x2 * self.scale + self.scale // 2
            y2_large = (grid_height - 1 - y2) * self.scale + self.scale // 2

            # Fade effect
            alpha = (i + 1) / len(trajectory_points)
            faded_color = tuple(int(c * alpha) for c in color)

            cv2.line(img, (x1_large, y1_large),
                     (x2_large, y2_large), faded_color, 2)

    def draw_robot_fov(self, img, robot_id, grid_height):
        """Draw the field of view for a robot if enabled"""
        if not self.robot_grid_positions[robot_id]:
            return

        rx, ry = self.robot_grid_positions[robot_id]
        robot_angle = self.robot_angles[robot_id]
        color = self.robot_colors[robot_id]

        # Convert to display coordinates
        rx_large = rx * self.scale + self.scale // 2
        ry_large = (grid_height - 1 - ry) * self.scale + self.scale // 2

        # FOV parameters
        camera_fov = math.radians(60)
        camera_range = 8.0  # Match the mapper's max_range
        fov_half = camera_fov / 2

        # Calculate FOV boundaries
        left_angle = robot_angle - fov_half
        right_angle = robot_angle + fov_half

        max_range_pixels = int(camera_range / self.resolution * self.scale)

        # Draw FOV cone
        points = [(rx_large, ry_large)]

        # Add arc points
        num_arc_points = 20
        for i in range(num_arc_points + 1):
            angle = left_angle + (right_angle - left_angle) * i / num_arc_points
            x = int(rx_large + max_range_pixels * math.cos(angle))
            y = int(ry_large - max_range_pixels * math.sin(angle))
            points.append((x, y))

        # Draw filled polygon with transparency (handled by overlay)
        cv2.fillPoly(img, [np.array(points)], color)

    def destroy_node(self):
        """Clean up on shutdown"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Robot configurations - must match map merger and mapper
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        1: {'position': [-1.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 2: {'position': [5.0, 5.0, 0.5], 'orientation': [0.0, 0.0, 0.0]}
    }

    node = MultiRobot2DVisualizer(robot_configs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()