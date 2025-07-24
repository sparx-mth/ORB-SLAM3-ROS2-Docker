#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import threading
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree


class MultiRobotMapMerger(Node):
    """
    Dedicated node for merging maps from multiple robots.
    Subscribes to landmark topics published by landmark_publisher_node.py
    """

    def __init__(self, robot_configs):
        super().__init__('multi_robot_map_merger')

        self.robot_configs = robot_configs
        self.robot_ids = list(robot_configs.keys())

        # Configuration
        self.enable_logging = True
        self.merge_distance_threshold = 0.1  # meters
        self.min_points_for_merge = 100

        # Data storage
        self.robot_poses = {rid: None for rid in robot_configs.keys()}
        self.raw_landmarks = {rid: np.empty((0, 3)) for rid in robot_configs.keys()}
        self.landmarks_received = {rid: False for rid in robot_configs.keys()}

        # Transformation matrices
        self.robot_transforms = {}
        for robot_id, config in robot_configs.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config.get('orientation', [0, 0, 0])
            )

        # Thread safety
        self.data_lock = threading.Lock()

        # Publishers
        self.merged_map_pub = self.create_publisher(
            PointCloud2, '/merged_map', 10
        )

        # Subscribe to robot poses
        for robot_id in robot_configs.keys():
            pose_topic = f'/robot_{robot_id}/robot_pose_slam'
            self.create_subscription(
                PoseStamped,
                pose_topic,
                self.create_pose_callback(robot_id),
                10
            )
            self.get_logger().info(f'Subscribed to pose topic: {pose_topic}')

        # Subscribe to landmark topics from landmark_publisher_node
        for robot_id in robot_configs.keys():
            landmark_topic = f'/robot_{robot_id}/orb_slam3/landmarks_raw'
            self.create_subscription(
                PointCloud2,
                landmark_topic,
                self.create_landmark_callback(robot_id),
                10
            )
            self.get_logger().info(f'Subscribed to landmark topic: {landmark_topic}')

        # Timer for publishing merged map
        self.create_timer(1.0, self.publish_merged_map)

        # Timer for status updates
        self.create_timer(5.0, self.print_status)

        self.get_logger().info(f'Map Merger initialized for robots: {self.robot_ids}')

    def create_transformation_matrix(self, position, orientation):
        """Create 4x4 transformation matrix"""
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def create_pose_callback(self, robot_id):
        """Factory for pose callbacks"""

        def callback(msg):
            with self.data_lock:
                # Get pose in robot's local frame
                local_pose = np.array([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    1.0
                ])

                # Transform to global frame
                global_pose = self.robot_transforms[robot_id] @ local_pose
                self.robot_poses[robot_id] = global_pose[:3]

        return callback

    def create_landmark_callback(self, robot_id):
        """Factory for landmark callbacks"""

        def callback(msg):
            try:
                points = list(pc2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ))

                if points:
                    with self.data_lock:
                        # Transform points to global frame
                        local_points = np.array([[p[0], p[1], p[2]] for p in points])
                        global_points = self.transform_points_to_global(
                            local_points, robot_id
                        )
                        self.raw_landmarks[robot_id] = global_points
                        self.landmarks_received[robot_id] = True

                    if self.enable_logging:
                        self.get_logger().info(
                            f'Updated landmarks from robot_{robot_id}: {len(points)} points'
                        )
            except Exception as e:
                self.get_logger().error(
                    f'Error processing landmarks from robot_{robot_id}: {e}'
                )

        return callback

    def transform_points_to_global(self, local_points, robot_id):
        """Transform points from robot frame to global frame"""
        if len(local_points) == 0:
            return local_points

        transform_matrix = self.robot_transforms[robot_id]

        # Add homogeneous coordinate
        ones = np.ones((local_points.shape[0], 1))
        local_homo = np.hstack([local_points, ones])

        # Transform
        global_homo = (transform_matrix @ local_homo.T).T

        # Return 3D points
        return global_homo[:, :3]

    def merge_maps(self):
        """Merge maps from all robots and remove duplicates"""
        all_points = []
        all_colors = []

        # Define robot colors
        robot_colors = {
            0: [0, 0, 255],  # Blue
            1: [0, 255, 0],  # Green
            2: [255, 0, 0],  # Red
        }

        with self.data_lock:
            # Collect all points with robot IDs
            for robot_id in self.robot_ids:
                if self.raw_landmarks[robot_id].shape[0] > 0:
                    points = self.raw_landmarks[robot_id]
                    color = robot_colors.get(robot_id, [128, 128, 128])

                    all_points.append(points)
                    all_colors.extend([color] * len(points))

        if not all_points:
            return None, None

        # Concatenate all points
        merged_points = np.vstack(all_points)
        merged_colors = np.array(all_colors)

        # Remove duplicates using KDTree
        if len(merged_points) > self.min_points_for_merge:
            tree = cKDTree(merged_points)

            # Find points that are too close to each other
            duplicate_mask = np.ones(len(merged_points), dtype=bool)

            for i in range(len(merged_points)):
                if duplicate_mask[i]:
                    # Find neighbors within threshold
                    neighbors = tree.query_ball_point(
                        merged_points[i],
                        self.merge_distance_threshold
                    )
                    # Keep only the first point in each cluster
                    for j in neighbors[1:]:
                        duplicate_mask[j] = False

            # Apply mask
            merged_points = merged_points[duplicate_mask]
            merged_colors = merged_colors[duplicate_mask]

        return merged_points, merged_colors

    def publish_merged_map(self):
        """Publish the merged point cloud"""
        merged_points, merged_colors = self.merge_maps()

        if merged_points is None or len(merged_points) == 0:
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        # Pack point data
        cloud_data = []
        for i, point in enumerate(merged_points):
            # Pack color as RGB
            r, g, b = merged_colors[i]
            rgb = (r << 16) | (g << 8) | b

            cloud_data.append(struct.pack('fffi',
                                          point[0], point[1], point[2], rgb
                                          ))

        # Create message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(merged_points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 bytes each for x, y, z, rgb
        msg.row_step = 16 * len(merged_points)
        msg.data = b''.join(cloud_data)
        msg.is_dense = True

        self.merged_map_pub.publish(msg)

        if self.enable_logging:
            # Count points per robot
            robot_counts = {}
            for rid in self.robot_ids:
                robot_counts[rid] = len(self.raw_landmarks[rid])

            self.get_logger().info(
                f'Published merged map: {len(merged_points)} points '
                f'(from {robot_counts})'
            )

    def print_status(self):
        """Print current status"""
        with self.data_lock:
            total_points = 0
            status_parts = []

            for robot_id in self.robot_ids:
                count = len(self.raw_landmarks[robot_id])
                total_points += count
                received = "✓" if self.landmarks_received[robot_id] else "✗"
                status_parts.append(f"R{robot_id}:{count}{received}")

            status = f"Map Merger Status: {' | '.join(status_parts)} | Total: {total_points}"
            self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)

    # Robot configurations - must match those in launch file
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        1: {'position': [-1.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 2: {'position': [5.0, 5.0, 0.5], 'orientation': [0.0, 0.0, 0.0]}
    }

    node = MultiRobotMapMerger(robot_configs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()