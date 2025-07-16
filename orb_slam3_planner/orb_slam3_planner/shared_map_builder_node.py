#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math


class SharedMapBuilderNode(Node):
    """
    A ROS2 node that subscribes to a merged global point cloud from multiple robots
    and converts it into a 2D occupancy grid for shared exploration and planning.
    """

    def __init__(self):
        super().__init__('shared_map_builder_node')

        # Parameters (same as in single-agent MapBuilder)
        self.cell_size = 0.25        # Grid resolution (m per cell)
        self.map_range = 20.0        # Map range from center (meters)
        self.grid_size = int(2 * self.map_range / self.cell_size)

        self.height_min = 0.1        # Min Z height to consider
        self.height_max = 2.0        # Max Z height to consider
        self.camera_range = 10.0     # Max distance for a point to be included

        self.obstacle_prob_increment = 0.2
        self.free_prob_decrement = -0.05
        self.occupied_threshold = 0.75
        self.free_threshold = 0.35
        self.freeze_update_count = 8

        # Probability grid
        self.occupancy_prob = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)
        self.update_count = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

        # ROS interfaces
        self.create_subscription(PointCloud2, '/merged_map', self.pointcloud_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/global_occupancy_grid', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Shared Map Builder Node started.')

    def pointcloud_callback(self, msg):
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"Failed to parse point cloud: {e}")
            return

        occupied_cells = set()

        for x, y, z in points:
            if not (self.height_min <= z <= self.height_max):
                continue

            dist = math.sqrt(x**2 + y**2)
            if dist > self.camera_range:
                continue

            grid_x = int((x + self.map_range) / self.cell_size)
            grid_y = int((y + self.map_range) / self.cell_size)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                occupied_cells.add((grid_x, grid_y))
                self.update_cell_probability(grid_x, grid_y, self.obstacle_prob_increment)

        self.update_free_space(occupied_cells)
        self.decay_probabilities()

    def update_cell_probability(self, x, y, prob_change):
        old_prob = self.occupancy_prob[y, x]

        if prob_change > 0:
            new_prob = old_prob + prob_change * (1 - old_prob)
        else:
            new_prob = old_prob + prob_change * old_prob

        self.occupancy_prob[y, x] = np.clip(new_prob, 0.01, 0.99)
        self.update_count[y, x] += 1

    def update_free_space(self, occupied_cells):
        for (gx, gy) in occupied_cells:
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        self.update_cell_probability(nx, ny, self.free_prob_decrement)

    def decay_probabilities(self):
        decay_factor = 0.99
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if self.update_count[y, x] < self.freeze_update_count:
                    old_prob = self.occupancy_prob[y, x]
                    self.occupancy_prob[y, x] = 0.5 + (old_prob - 0.5) * decay_factor

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.map_range
        msg.info.origin.position.y = -self.map_range

        grid_data = []
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                prob = self.occupancy_prob[y, x]
                count = self.update_count[y, x]
                if count < 2:
                    grid_data.append(-1)
                elif prob > self.occupied_threshold:
                    grid_data.append(100)
                elif prob < self.free_threshold:
                    grid_data.append(0)
                else:
                    grid_data.append(-1)

        msg.data = grid_data
        self.map_pub.publish(msg)
        self.get_logger().info(f"Published global occupancy grid.")


def main(args=None):
    rclpy.init(args=args)
    node = SharedMapBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
