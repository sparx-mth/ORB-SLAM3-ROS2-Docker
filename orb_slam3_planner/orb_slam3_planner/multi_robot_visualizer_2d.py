#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import cv2
import math
from collections import deque
import time


class MultiRobot2DVisualizer(Node):
    """
    2D visualization of the multi-robot exploration system.
    Optimized version with configurable features.
    """

    def __init__(self):
        super().__init__('multi_robot_2d_visualizer')

        # Configuration flags
        self.enable_visualization = True  # Master switch for visualization
        self.enable_fov_visualization = False  # Disable FOV cone drawing for performance
        self.enable_ray_tracing = False  # Disable ray tracing visualization for performance
        self.enable_trajectories = True  # Enable trajectory tracking
        self.enable_logging = True  # Enable logging

        # Visualization parameters
        self.scale = 7  # Scale factor for display
        self.window_name = 'Multi-Robot Exploration Map'

        # Robot IDs and colors - matching multi_robot_map_builder.py configuration
        self.robot_ids = [0, 1]  # Update this to [0, 1, 2] if using 3 robots
        self.robot_colors = {
            0: (255, 0, 0),  # Blue
            1: (0, 255, 0),  # Green
            2: (0, 0, 255),  # Red
        }

        # Robot states
        self.robot_positions = {rid: None for rid in self.robot_ids}
        self.robot_angles = {rid: 0.0 for rid in self.robot_ids}
        self.robot_goals = {rid: None for rid in self.robot_ids}

        # Map data
        self.occupancy_grid = None
        self.grid_size = 0
        self.cell_size = 0.25

        # Performance tracking
        self.last_update_time = time.time()
        self.frame_count = 0

        # Only initialize trajectories if enabled
        if self.enable_trajectories:
            self.trajectories = {rid: deque(maxlen=400) for rid in self.robot_ids}

        # Subscriptions
        self.create_subscription(
            OccupancyGrid, '/shared_occupancy_grid',
            self.map_callback, 10
        )

        for robot_id in self.robot_ids:
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

        if self.enable_logging:
            self.get_logger().info('Multi-Robot 2D Visualizer started')

    def create_position_callback(self, robot_id):
        """Factory for position callbacks"""

        def callback(msg):
            self.robot_positions[robot_id] = (int(msg.x), int(msg.y))
            self.robot_angles[robot_id] = msg.z

            # Only track trajectories if enabled
            if self.enable_trajectories and self.robot_positions[robot_id]:
                self.trajectories[robot_id].append(self.robot_positions[robot_id])

        return callback

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

        width = msg.info.width
        height = msg.info.height
        self.grid_size = width
        self.cell_size = msg.info.resolution

        # Convert occupancy grid to numpy array
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.occupancy_grid = grid_data

        # Create colored image
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Color coding for map
        img[grid_data == -1] = (128, 128, 128)  # Unknown = Gray
        img[grid_data == 0] = (255, 255, 255)  # Free = White
        img[grid_data == 100] = (0, 0, 255)  # Obstacle = Red

        # Flip image (ROS coordinate system)
        img = cv2.flip(img, 0)

        # Scale up for better visibility
        img_large = cv2.resize(img, (width * self.scale, height * self.scale),
                               interpolation=cv2.INTER_NEAREST)

        # Draw trajectories if enabled
        if self.enable_trajectories:
            for robot_id in self.robot_ids:
                if robot_id in self.trajectories and len(self.trajectories[robot_id]) > 1:
                    self.draw_trajectory(img_large, robot_id, height)

        # Draw FOV if enabled
        if self.enable_fov_visualization:
            overlay = img_large.copy()
            for robot_id in self.robot_ids:
                if self.robot_positions[robot_id]:
                    self.draw_robot_fov(overlay, robot_id, height)
            cv2.addWeighted(overlay, 0.3, img_large, 0.7, 0, img_large)

        # Draw goals
        for robot_id in self.robot_ids:
            if self.robot_goals[robot_id]:
                gx, gy = self.robot_goals[robot_id]
                if 0 <= gx < width and 0 <= gy < height:
                    gx_large = gx * self.scale
                    gy_large = (height - 1 - gy) * self.scale

                    # Draw goal as circle with robot's color
                    color = self.robot_colors[robot_id]
                    cv2.circle(img_large, (gx_large, gy_large),
                               radius=self.scale * 2, color=(0, 0, 0), thickness=2)

                    # Add text label
                    cv2.putText(img_large, f'G{robot_id}',
                                (gx_large + 10, gy_large - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Draw robots
        for robot_id in self.robot_ids:
            if self.robot_positions[robot_id]:
                rx, ry = self.robot_positions[robot_id]
                if 0 <= rx < width and 0 <= ry < height:
                    rx_large = rx * self.scale
                    ry_large = (height - 1 - ry) * self.scale

                    # Draw robot as circle
                    color = self.robot_colors[robot_id]
                    cv2.circle(img_large, (rx_large, ry_large),
                               radius=max(5, self.scale), color=color, thickness=-1)

                    # Draw direction arrow
                    if self.robot_angles[robot_id] is not None:
                        arrow_len = self.scale * 3
                        dx = int(arrow_len * math.cos(self.robot_angles[robot_id]))
                        dy = int(-arrow_len * math.sin(self.robot_angles[robot_id]))

                        cv2.arrowedLine(img_large, (rx_large, ry_large),
                                        (rx_large + dx, ry_large + dy),
                                        (0, 0, 0), 2, tipLength=0.4)

                    # Draw robot ID
                    cv2.putText(img_large, f'R{robot_id}',
                                (rx_large + 10, ry_large + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Add info panel
        cv2.putText(img_large, 'Multi-Robot Explorer',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(img_large, 'White=Free, Red=Obstacle, Gray=Unknown',
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Show robot positions
        y_offset = 65
        for robot_id in self.robot_ids:
            if self.robot_positions[robot_id]:
                rx, ry = self.robot_positions[robot_id]
                color = self.robot_colors[robot_id]
                cv2.putText(img_large, f'R{robot_id}: ({rx}, {ry})',
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                y_offset += 20

        # Display
        cv2.imshow(self.window_name, img_large)
        cv2.waitKey(1)

        # FPS tracking
        self.frame_count += 1
        current_time = time.time()
        if self.enable_logging and current_time - self.last_update_time > 5.0:
            fps = self.frame_count / (current_time - self.last_update_time)
            self.get_logger().info(f'Visualization FPS: {fps:.1f}')
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
            x1_large = x1 * self.scale
            y1_large = (grid_height - 1 - y1) * self.scale
            x2_large = x2 * self.scale
            y2_large = (grid_height - 1 - y2) * self.scale

            # Fade effect
            alpha = (i + 1) / len(trajectory_points)
            faded_color = tuple(int(c * alpha) for c in color)

            cv2.line(img, (x1_large, y1_large),
                     (x2_large, y2_large), faded_color, 2)

    def draw_robot_fov(self, img, robot_id, grid_height):
        """Draw the field of view for a robot if enabled"""
        if not self.robot_positions[robot_id]:
            return

        rx, ry = self.robot_positions[robot_id]
        robot_angle = self.robot_angles[robot_id]
        color = self.robot_colors[robot_id]

        # Convert to display coordinates
        rx_large = rx * self.scale
        ry_large = (grid_height - 1 - ry) * self.scale

        # Simple FOV cone
        camera_fov = math.radians(60)
        camera_range = 10.0
        fov_half = camera_fov / 2
        left_angle = robot_angle - fov_half
        right_angle = robot_angle + fov_half

        max_range_pixels = int(camera_range / self.cell_size * self.scale)

        # Left boundary
        left_x = int(rx_large + max_range_pixels * math.cos(left_angle))
        left_y = int(ry_large - max_range_pixels * math.sin(left_angle))
        cv2.line(img, (rx_large, ry_large), (left_x, left_y), color, 1)

        # Right boundary
        right_x = int(rx_large + max_range_pixels * math.cos(right_angle))
        right_y = int(ry_large - max_range_pixels * math.sin(right_angle))
        cv2.line(img, (rx_large, ry_large), (right_x, right_y), color, 1)

        # Draw ray traces if enabled
        if self.enable_ray_tracing and self.occupancy_grid is not None:
            self.draw_ray_traces(img, robot_id, grid_height)

    def draw_ray_traces(self, img, robot_id, grid_height):
        """Draw ray traces if enabled"""
        if not self.robot_positions[robot_id]:
            return

        rx, ry = self.robot_positions[robot_id]
        robot_angle = self.robot_angles[robot_id]
        color = self.robot_colors[robot_id]

        # Minimal ray tracing
        camera_fov = math.radians(60)
        camera_range = 10.0
        fov_half = camera_fov / 2
        num_rays = 10
        max_range_cells = int(camera_range / self.cell_size)

        for i in range(0, num_rays, 2):  # Skip every other ray
            angle_offset = -fov_half + (i * camera_fov / (num_rays - 1))
            ray_angle = robot_angle + angle_offset

            for dist in range(1, max_range_cells):
                cell_x = int(rx + dist * math.cos(ray_angle))
                cell_y = int(ry + dist * math.sin(ray_angle))

                if not (0 <= cell_x < self.grid_size and 0 <= cell_y < self.grid_size):
                    break

                if self.occupancy_grid[cell_y, cell_x] == 100:  # Occupied
                    start_x = rx * self.scale
                    start_y = (grid_height - 1 - ry) * self.scale
                    end_x = cell_x * self.scale
                    end_y = (grid_height - 1 - cell_y) * self.scale

                    cv2.line(img, (start_x, start_y), (end_x, end_y),
                             tuple(c // 3 for c in color), 1)
                    break

    def destroy_node(self):
        """Clean up on shutdown"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobot2DVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()