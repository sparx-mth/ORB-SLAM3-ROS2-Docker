#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import numpy as np
import cv2


class GlobalMapVisualizer(Node):
    """
    A ROS2 node to visualize the shared global occupancy grid and all robot positions/goals.
    """

    def __init__(self):
        super().__init__('global_map_visualizer')

        # Parameters
        self.scale = 7  # Zoom factor for display
        self.window_name = 'Global Map Viewer'

        # State storage
        self.robot_positions = {}   # robot_id: (x, y, theta)
        self.robot_goals = {}       # robot_id: (x, y)

        # Map
        self.grid_data = None
        self.grid_width = None
        self.grid_height = None

        # ROS setup
        self.create_subscription(OccupancyGrid, '/global_occupancy_grid', self.map_callback, 10)

        for robot_id in range(10):  # Support up to 10 robots
            ns = f'/robot_{robot_id}'
            self.create_subscription(Point, f'{ns}/robot_grid_pos', self.make_robot_callback(robot_id), 10)
            self.create_subscription(Point, f'{ns}/goal_grid_pos', self.make_goal_callback(robot_id), 10)

        self.get_logger().info('Global Map Visualizer Node started.')

    def make_robot_callback(self, robot_id):
        def callback(msg):
            self.robot_positions[robot_id] = (int(msg.x), int(msg.y), float(msg.z))
        return callback

    def make_goal_callback(self, robot_id):
        def callback(msg):
            self.robot_goals[robot_id] = (int(msg.x), int(msg.y))
        return callback

    def map_callback(self, msg):
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.grid_data = np.array(msg.data, dtype=np.int8).reshape((self.grid_height, self.grid_width))
        self.render()

    def render(self):
        if self.grid_data is None:
            return

        # Color grid
        img = np.zeros((self.grid_height, self.grid_width, 3), dtype=np.uint8)
        img[self.grid_data == -1] = (128, 128, 128)
        img[self.grid_data == 0] = (0, 255, 0)
        img[self.grid_data == 100] = (0, 0, 255)

        img = cv2.flip(img, 0)
        img_large = cv2.resize(img, (self.grid_width * self.scale, self.grid_height * self.scale), interpolation=cv2.INTER_NEAREST)

        # Draw robots
        for robot_id, (x, y, theta) in self.robot_positions.items():
            x_px = x * self.scale
            y_px = (self.grid_height - 1 - y) * self.scale
            cv2.circle(img_large, (x_px, y_px), radius=6, color=(255, 255, 255), thickness=-1)

            # Direction
            arrow_len = self.scale * 3
            dx = int(arrow_len * math.cos(theta))
            dy = int(-arrow_len * math.sin(theta))
            cv2.arrowedLine(img_large, (x_px, y_px), (x_px + dx, y_px + dy), (0, 255, 255), 2, tipLength=0.4)

            cv2.putText(img_large, f'R{robot_id}', (x_px + 10, y_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Draw goals
        for robot_id, (gx, gy) in self.robot_goals.items():
            gx_px = gx * self.scale
            gy_px = (self.grid_height - 1 - gy) * self.scale
            cv2.circle(img_large, (gx_px, gy_px), radius=self.scale * 2, color=(0, 255, 255), thickness=2)
            cv2.putText(img_large, f'G{robot_id}', (gx_px + 10, gy_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        # Title and legend
        cv2.putText(img_large, 'Global Occupancy Grid', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img_large, 'Green=Free, Red=Obstacle, Gray=Unknown, White=Robot, Yellow=Goal',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.imshow(self.window_name, img_large)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
