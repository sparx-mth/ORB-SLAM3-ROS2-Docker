#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import math
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import threading
import time


class MultiRobotMapBuilder(Node):
    """
    Builds a shared 2D occupancy map from multiple robots' point clouds and poses.
    Optimized version that matches single-robot performance.
    """

    def __init__(self, robot_configs):
        super().__init__('multi_robot_map_builder')

        # Configuration flags
        self.enable_logging = False  # Set to True for debug messages
        self.enable_neighbor_updates = False  # Disable neighbor cell updates for performance
        self.enable_decay = False  # Disable probability decay for performance

        self.robot_configs = robot_configs
        self.robot_ids = list(robot_configs.keys())

        # Map parameters (same as single robot)
        self.cell_size = 0.25
        self.map_range = 40.0
        self.grid_size = int(2 * self.map_range / self.cell_size)

        # Shared occupancy grid
        self.occupancy_prob = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)
        self.update_count = np.zeros((self.grid_size, self.grid_size), dtype=np.int32)

        # Height filtering
        self.height_min = 0.1
        self.height_max = 2.0

        # Probability updates
        self.obstacle_prob_increment = 0.2
        self.free_prob_decrement = -0.05
        self.occupied_threshold = 0.75
        self.free_threshold = 0.35
        self.freeze_update_count = 8

        # Sensor parameters
        self.camera_fov = math.radians(60)
        self.camera_range = 10.0
        self.min_points_for_obstacle = 20

        # Robot states
        self.robot_poses = {rid: None for rid in self.robot_ids}
        self.robot_angles = {rid: 0.0 for rid in self.robot_ids}
        self.robot_transforms = {}

        # Thread safety
        self.map_lock = threading.Lock()

        # Create transformation matrices
        for robot_id, config in robot_configs.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config.get('orientation', [0, 0, 0])
            )

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/shared_occupancy_grid', 10)
        self.robot_grid_pubs = {}

        for robot_id in self.robot_ids:
            self.robot_grid_pubs[robot_id] = self.create_publisher(
                Point, f'/robot_{robot_id}/grid_position', 10
            )

        # Subscriptions
        for robot_id in self.robot_ids:
            # Subscribe to poses
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/robot_pose_slam',
                self.create_pose_callback(robot_id),
                10
            )

            # Subscribe to point clouds
            self.create_subscription(
                PointCloud2,
                f'/robot_{robot_id}/orb_slam3/landmarks_raw',
                self.create_pointcloud_callback(robot_id),
                10
            )

        # Timer for map publishing only
        self.create_timer(0.5, self.publish_map)

        if self.enable_decay:
            self.create_timer(2.0, self.decay_probabilities)

        if self.enable_logging:
            self.get_logger().info(f'Multi-Robot Map Builder initialized for robots: {self.robot_ids}')

    def create_transformation_matrix(self, position, orientation):
        """Create 4x4 transformation matrix"""
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def create_pose_callback(self, robot_id):
        """Factory function to create pose callbacks for each robot"""

        def callback(msg):
            # Get pose in robot's local frame
            local_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                1.0
            ])

            # Transform to global frame
            global_pose = self.robot_transforms[robot_id] @ local_pose

            # Extract global position
            world_x = global_pose[0]
            world_y = global_pose[1]

            # Convert to grid coordinates
            grid_x = int((world_x + self.map_range) / self.cell_size)
            grid_y = int((world_y + self.map_range) / self.cell_size)

            # Extract orientation
            q = msg.pose.orientation
            local_angle = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # Apply rotation from transformation
            rot_matrix = self.robot_transforms[robot_id][:3, :3]
            transform_yaw = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])
            global_angle = local_angle + transform_yaw

            self.robot_poses[robot_id] = (grid_x, grid_y)
            self.robot_angles[robot_id] = global_angle

            # Publish grid position
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid_msg = Point()
                grid_msg.x = float(grid_x)
                grid_msg.y = float(grid_y)
                grid_msg.z = float(global_angle)
                self.robot_grid_pubs[robot_id].publish(grid_msg)

        return callback

    def create_pointcloud_callback(self, robot_id):
        """Factory function to create point cloud callbacks for each robot"""

        def callback(msg):
            if self.robot_poses[robot_id] is None:
                return

            try:
                points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            except Exception as e:
                if self.enable_logging:
                    self.get_logger().error(f'Error reading point cloud from robot_{robot_id}: {e}')
                return

            if not points:
                return

            # Get robot's position for calculations
            robot_grid_x, robot_grid_y = self.robot_poses[robot_id]
            robot_angle = self.robot_angles[robot_id]

            # Process points like single robot - no complex transformations
            cell_points = {}

            for x, y, z in points:
                # Height filter
                if not (self.height_min <= z <= self.height_max):
                    continue

                # Transform point to global frame
                local_point = np.array([x, y, z, 1.0])
                global_point = self.robot_transforms[robot_id] @ local_point

                # Get global coordinates
                global_x = global_point[0]
                global_y = global_point[1]

                # Convert to grid
                grid_x = int((global_x + self.map_range) / self.cell_size)
                grid_y = int((global_y + self.map_range) / self.cell_size)

                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    cell_points.setdefault((grid_x, grid_y), []).append((global_x, global_y, z))

            # Update occupancy grid - simplified like single robot
            occupied_cells = set()

            for (gx, gy), pts in cell_points.items():
                if len(pts) >= self.min_points_for_obstacle:
                    prob_increase = min(self.obstacle_prob_increment * len(pts), 0.5)
                    self.update_cell_probability(gx, gy, prob_increase)
                    occupied_cells.add((gx, gy))

                    # Optional: Update neighbors
                    if self.enable_neighbor_updates:
                        for dx in [-1, 0, 1]:
                            for dy in [-1, 0, 1]:
                                nx, ny = gx + dx, gy + dy
                                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                                    self.update_cell_probability(nx, ny, prob_increase * 0.5)

            # Update free space - simplified
            if self.robot_poses[robot_id]:
                self.update_free_space_simple(robot_id, robot_grid_x, robot_grid_y,
                                              robot_angle, occupied_cells)

        return callback

    def update_cell_probability(self, x, y, prob_change):
        """Update a cell's occupancy probability"""
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return

        old_prob = self.occupancy_prob[y, x]

        if prob_change > 0:
            new_prob = old_prob + prob_change * (1 - old_prob)
        else:
            new_prob = old_prob + prob_change * old_prob

        self.occupancy_prob[y, x] = np.clip(new_prob, 0.01, 0.99)
        self.update_count[y, x] += 1

    def update_free_space_simple(self, robot_id, robot_x, robot_y, robot_angle, occupied_cells):
        """Simplified free space update matching single robot performance"""
        # Only process occupied cells within FOV
        for gx, gy in occupied_cells:
            dx = gx - robot_x
            dy = gy - robot_y
            angle_to_obstacle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(angle_to_obstacle - robot_angle)

            if abs(angle_diff) <= self.camera_fov / 2:
                cells_on_ray = self.bresenham_line(robot_x, robot_y, gx, gy)
                for (x, y) in cells_on_ray[:-1]:
                    if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                        self.update_cell_probability(x, y, self.free_prob_decrement * 1.5)

        # Simple FOV free space marking
        fov_half = self.camera_fov / 2
        num_rays = int(self.camera_fov / math.radians(5))
        max_range_cells = int(self.camera_range / self.cell_size)

        for i in range(num_rays):
            angle_offset = -fov_half + (i * self.camera_fov / (num_rays - 1))
            angle = robot_angle + angle_offset

            for dist in range(1, max_range_cells):
                x = int(robot_x + dist * math.cos(angle))
                y = int(robot_y + dist * math.sin(angle))

                if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
                    break

                if self.occupancy_prob[y, x] > self.occupied_threshold and \
                        self.update_count[y, x] > 3:
                    break

                self.update_cell_probability(x, y, self.free_prob_decrement)

    def bresenham_line(self, start_x, start_y, end_x, end_y):
        """Bresenham's line algorithm"""
        cells = []
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        x, y = start_x, start_y
        x_inc = 1 if end_x > start_x else -1
        y_inc = 1 if end_y > start_y else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        while True:
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                cells.append((x, y))
            if x == end_x and y == end_y:
                break
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

        return cells

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def decay_probabilities(self):
        """Slowly decay probabilities - only if enabled"""
        if not self.enable_decay:
            return

        decay_factor = 0.99

        with self.map_lock:
            for y in range(self.grid_size):
                for x in range(self.grid_size):
                    if self.update_count[y, x] < self.freeze_update_count:
                        old_prob = self.occupancy_prob[y, x]
                        self.occupancy_prob[y, x] = 0.5 + (old_prob - 0.5) * decay_factor

    def publish_map(self):
        """Publish the shared occupancy grid"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.map_range
        msg.info.origin.position.y = -self.map_range

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
        """Convert probability to occupancy value"""
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return -1

        prob = self.occupancy_prob[y, x]
        updates = self.update_count[y, x]

        if updates < 2:
            return -1
        if prob > self.occupied_threshold:
            return 1
        if prob < self.free_threshold:
            return 0
        return -1

    def get_robot_positions(self):
        """Get current robot positions in grid coordinates"""
        return {rid: pos for rid, pos in self.robot_poses.items() if pos is not None}


def main(args=None):
    rclpy.init(args=args)

    # Robot configurations
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        1: {'position': [-1.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 2: {'position': [5.0, 5.0, 0.5], 'orientation': [0.0, 0.0, 0.0]}
    }

    node = MultiRobotMapBuilder(robot_configs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()