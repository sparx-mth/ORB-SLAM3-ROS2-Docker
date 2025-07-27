#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import math
from collections import defaultdict
import threading


class EfficientOccupancyGridMapper(Node):
    """
    Efficient real-time occupancy grid mapper for multi-robot SLAM.
    Converts 3D point clouds to 2D occupancy grid with visibility constraints.
    """

    def __init__(self, robot_configs):
        super().__init__('efficient_occupancy_grid_mapper')

        self.robot_configs = robot_configs
        self.robot_ids = list(robot_configs.keys())

        # ======================
        # Map Parameters
        # ======================
        self.resolution = 0.5  # 10cm cells for good balance
        self.map_size_meters = 30.0  # Total map size (30m x 30m)
        self.grid_size = int(self.map_size_meters / self.resolution)
        self.origin_offset = self.map_size_meters / 2.0

        # Height filter for 2D projection
        self.z_min = 0.1  # Minimum height to consider
        self.z_max = 2.0  # Maximum height to consider

        # Occupancy thresholds
        self.points_threshold = 3  # Min points in cell to mark as occupied
        self.max_range = 8.0  # Maximum sensor range in meters

        # ======================
        # Data Structures
        # ======================
        # Main occupancy grid: 0=free, 1=occupied, -1=unknown
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Point count grid for efficient density calculation
        self.point_count = np.zeros((self.grid_size, self.grid_size), dtype=np.uint16)

        # Robot states
        self.robot_poses = {rid: None for rid in self.robot_ids}
        self.robot_grid_poses = {rid: None for rid in self.robot_ids}
        self.robot_headings = {rid: 0.0 for rid in self.robot_ids}

        # Transformation matrices
        self.robot_transforms = {}
        for robot_id, config in robot_configs.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config.get('orientation', [0, 0, 0])
            )

        # Thread safety
        self.lock = threading.Lock()

        # ======================
        # Publishers & Subscribers
        # ======================
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/occupancy_grid', 10
        )

        # Subscribe to merged point cloud
        self.create_subscription(
            PointCloud2,
            '/merged_map',
            self.point_cloud_callback,
            10
        )

        # Subscribe to robot poses
        for robot_id in self.robot_ids:
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/robot_pose_slam',
                self.create_pose_callback(robot_id),
                10
            )

        # Timer for map publishing
        self.create_timer(0.5, self.publish_map)  # 2Hz publishing

        self.get_logger().info("Efficient Occupancy Grid Mapper initialized")

    def create_transformation_matrix(self, position, orientation):
        """Create 4x4 transformation matrix"""
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        gx = int((x + self.origin_offset) / self.resolution)
        gy = int((y + self.origin_offset) / self.resolution)
        return gx, gy

    def is_valid_grid_pos(self, gx, gy):
        """Check if grid position is valid"""
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def create_pose_callback(self, robot_id):
        """Create pose callback for each robot"""

        def callback(msg):
            # Transform to global frame
            local_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                1.0
            ])
            global_pose = self.robot_transforms[robot_id] @ local_pose

            # Store world position
            world_x, world_y = global_pose[0], global_pose[1]
            self.robot_poses[robot_id] = (world_x, world_y)

            # Convert to grid
            gx, gy = self.world_to_grid(world_x, world_y)
            if self.is_valid_grid_pos(gx, gy):
                self.robot_grid_poses[robot_id] = (gx, gy)

            # Calculate heading
            q = msg.pose.orientation
            local_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # Apply transformation rotation
            rot_matrix = self.robot_transforms[robot_id][:3, :3]
            transform_yaw = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])
            self.robot_headings[robot_id] = local_yaw + transform_yaw

        return callback

    def point_cloud_callback(self, msg):
        """Process merged point cloud efficiently"""
        try:
            # Extract points
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if self.z_min <= p[2] <= self.z_max:  # Height filter
                    points.append((p[0], p[1]))

            if not points:
                return

            with self.lock:
                # Reset point count
                self.point_count.fill(0)

                # Count points per cell (very fast operation)
                for x, y in points:
                    gx, gy = self.world_to_grid(x, y)
                    if self.is_valid_grid_pos(gx, gy):
                        self.point_count[gy, gx] += 1

                # Mark cells as occupied based on point density
                occupied_cells = set()
                for gy in range(self.grid_size):
                    for gx in range(self.grid_size):
                        if self.point_count[gy, gx] >= self.points_threshold:
                            self.occupancy_grid[gy, gx] = 1
                            occupied_cells.add((gx, gy))

                # Update free space using efficient ray casting
                self.update_free_space_fast(occupied_cells)

        except Exception as e:
            self.get_logger().error(f"Error in point cloud callback: {e}")

    def update_free_space_fast(self, occupied_cells):
        """
        Efficient free space update using Bresenham's algorithm.
        Only traces rays from robots to occupied cells.
        """
        for robot_id, robot_grid_pos in self.robot_grid_poses.items():
            if robot_grid_pos is None:
                continue

            rx, ry = robot_grid_pos

            # Process only occupied cells within sensor range
            max_cells = int(self.max_range / self.resolution)

            for (ox, oy) in occupied_cells:
                # Check if occupied cell is within range
                dx = ox - rx
                dy = oy - ry
                dist_sq = dx * dx + dy * dy

                if dist_sq <= max_cells * max_cells:
                    # Trace ray from robot to occupied cell
                    self.trace_ray_bresenham(rx, ry, ox, oy)

    def trace_ray_bresenham(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm for ray tracing.
        Marks cells as free until hitting the target cell.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            # Don't mark the robot's cell or the final cell as free
            if (x, y) != (x0, y0) and (x, y) != (x1, y1):
                if self.is_valid_grid_pos(x, y) and self.occupancy_grid[y, x] != 1:
                    self.occupancy_grid[y, x] = 0  # Mark as free

            # Check if we've reached the end
            if x == x1 and y == y1:
                break

            # Bresenham's algorithm step
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_map(self):
        """Publish the occupancy grid"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.origin_offset
        msg.info.origin.position.y = -self.origin_offset
        msg.info.origin.position.z = 0.0

        # Convert to ROS format: unknown=-1, free=0, occupied=100
        with self.lock:
            data = []
            for y in range(self.grid_size):
                for x in range(self.grid_size):
                    val = self.occupancy_grid[y, x]
                    if val == -1:
                        data.append(-1)
                    elif val == 0:
                        data.append(0)
                    else:  # val == 1
                        data.append(100)

            msg.data = data

        self.map_pub.publish(msg)

        # Log statistics occasionally
        if hasattr(self, '_last_log_time'):
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_log_time > 5.0:  # Log every 5 seconds
                self.log_statistics()
                self._last_log_time = current_time
        else:
            self._last_log_time = self.get_clock().now().nanoseconds / 1e9

    def log_statistics(self):
        """Log map statistics"""
        with self.lock:
            unknown = np.sum(self.occupancy_grid == -1)
            free = np.sum(self.occupancy_grid == 0)
            occupied = np.sum(self.occupancy_grid == 1)
            total = self.grid_size * self.grid_size

            self.get_logger().info(
                f"Map stats - Unknown: {unknown} ({100 * unknown / total:.1f}%), "
                f"Free: {free} ({100 * free / total:.1f}%), "
                f"Occupied: {occupied} ({100 * occupied / total:.1f}%)"
            )


def main(args=None):
    rclpy.init(args=args)

    # Robot configurations - must match map merger
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        1: {'position': [-1.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 2: {'position': [5.0, 5.0, 0.5], 'orientation': [0.0, 0.0, 0.0]}
    }

    node = EfficientOccupancyGridMapper(robot_configs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()