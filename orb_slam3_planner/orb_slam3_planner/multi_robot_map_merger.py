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
import yaml

class MultiRobotMapMerger(Node):
    """
    A ROS2 node for centralized 3D map merging in multi-robot SLAM systems.

    This node receives local 3D landmark maps and SLAM poses from multiple robots,
    transforms them into a shared global frame, filters noise and duplicates,
    and publishes a unified global point cloud for use by other modules.

    Responsibilities:
    -----------------
    - Subscribes to:
        * /robot_<id>/orb_slam3/landmarks_raw (PointCloud2): Local 3D landmarks.
        * /robot_<id>/robot_pose_slam (PoseStamped): SLAM-based robot pose in local frame.
    - Maintains:
        * Static transform from each robot's local frame to global frame (based on config).
        * Per-robot landmark storage and SLAM pose tracking.
    - Processing:
        * Filters raw point clouds using radius-based and proximity filtering.
        * Transforms all points to a global frame using the robot-specific transform.
        * Merges and deduplicates points across robots using KD-tree queries.
    - Publishes:
        * /merged_map (PointCloud2): The combined global point cloud with per-robot color encoding.

    Notes:
    ------
    - Each robot must be configured via the `robot_configs` parameter, providing initial position and orientation.
    - Robot colors are hardcoded per ID for visual differentiation.
    - Point filtering removes redundant and isolated landmarks before merging.
    """

    def __init__(self):
        super().__init__('multi_robot_map_merger')

        # Declare and read robot_configs parameter
        self.declare_parameter('robot_configs', '{}')
        robot_configs_yaml = self.get_parameter('robot_configs').value
        self.robot_configs = yaml.safe_load(robot_configs_yaml)

        # Extract robot IDs from config
        self.robot_ids = list(self.robot_configs.keys())

        # Configuration
        self.enable_logging = False
        self.merge_distance_threshold = 0.1  # meters
        self.min_points_for_merge = 100

        # Data storage
        self.robot_poses = {rid: None for rid in self.robot_configs.keys()}
        self.raw_landmarks = {rid: np.empty((0, 3)) for rid in self.robot_configs.keys()}
        self.landmarks_received = {rid: False for rid in self.robot_configs.keys()}

        # Transformation matrices
        self.robot_transforms = {}
        for robot_id, config in self.robot_configs.items():
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
        for robot_id in self.robot_configs.keys():
            pose_topic = f'/robot_{robot_id}/robot_pose_slam'
            self.create_subscription(
                PoseStamped,
                pose_topic,
                self.create_pose_callback(robot_id),
                10
            )
            self.get_logger().info(f'Subscribed to pose topic: {pose_topic}')

        # Subscribe to landmark topics from landmark_publisher_node
        for robot_id in self.robot_configs.keys():
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
        # self.create_timer(5.0, self.print_status)

        self.get_logger().info(f'Map Merger initialized for robots: {self.robot_ids}')

        self.min_neighbors = 2  # Minimum neighbors for isolated point removal
        self.radius = 0.15  # Radius for isolated point removal

    def create_transformation_matrix(self, position, orientation):
        """
        Create a 4x4 homogeneous transformation matrix from position and Euler orientation.

        Args:
            position (list[float]): Translation vector [x, y, z].
            orientation (list[float]): Euler angles [roll, pitch, yaw] in radians.

        Returns:
            np.ndarray: 4x4 transformation matrix.
        """
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def create_pose_callback(self, robot_id):
        """
        Generate a ROS subscriber callback for robot poses.

        Converts local pose to global frame using robot-specific transform.

        Args:
            robot_id (int): The ID of the robot for which the callback is created.

        Returns:
            Callable: A function to handle `PoseStamped` messages.
        """

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
        """
        Generate a ROS subscriber callback for receiving raw landmark point clouds.

        Applies filtering and transformation to align landmarks with the global frame.

        Args:
            robot_id (int): The ID of the robot for which the callback is created.

        Returns:
            Callable: A function to handle `PointCloud2` messages.
        """

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
                        # Convert to NumPy array (Nx3) and ensure float64 dtype
                        points_np = np.array(local_points).astype(np.float64)
                        if np.isnan(points_np).any() or np.isinf(points_np).any():
                            self.get_logger().error("NaN or Inf detected in points_np!")
                            return points_np
                        # Pass NumPy arrays to your helper functions
                        pruned_points_np = self.remove_close_points(points_np)
                        # Check if pruned_points_np is empty before further processing
                        if pruned_points_np.shape[0] == 0:
                            # self.get_logger().info(f'[landmark callback] from robot_{robot_id}: 0 points after close point removal.')
                            return
                        final_points_np = self.remove_isolated_points(pruned_points_np)
                        # In handle_full_map_response, after final_points_np is ready:
                        if np.isnan(final_points_np).any():
                            self.get_logger().error(f"NaN values found in final_points_np for robot_{robot_id}!")

                        global_points = self.transform_points_to_global(
                            final_points_np, robot_id
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
        """
        Transform local 3D points from robot frame to global frame using robot's transform.

        Args:
            local_points (np.ndarray): Nx3 array of points in robot's local frame.
            robot_id (int): The ID of the robot.

        Returns:
            np.ndarray: Nx3 array of transformed points in global frame.
        """
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
        """
        Merge and deduplicate landmark maps from all robots.

        Returns:
            Tuple[np.ndarray, np.ndarray]:
                - Merged 3D points (Nx3 array).
                - Corresponding RGB color for each point (Nx3 array).
        """
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
            pairs = tree.query_pairs(self.merge_distance_threshold)
            # Mark duplicates for removal
            duplicate_mask = np.ones(len(merged_points), dtype=bool)
            for i, j in pairs:
                duplicate_mask[j] = False  # Keep the first, remove the second

            merged_points = merged_points[duplicate_mask]
            merged_colors = merged_colors[duplicate_mask]

        return merged_points, merged_colors

    def publish_merged_map(self):
        """
        Construct and publish the merged map as a PointCloud2 message on the `/merged_map` topic.

        Combines points from all robots and applies filtering before publishing.
        """
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
        """
        Print debug information about number of points received from each robot.
        (Disabled by default unless called manually.)
        """
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

    def remove_close_points(self, points_np, min_dist=0.0005):
        """
        Removes points that are too close to others, keeping only one in a cluster.

        Args:
            points_np (np.ndarray): Nx3 NumPy array of 3D points.
            min_dist (float): Minimum distance between points to keep.
            (Assumes self.tree is set)

        Returns:
            np.ndarray: Filtered NumPy array of unique points.
        """
        if points_np.shape[0] == 0:
            return points_np

        # Ensure the tree is built on the current set of points
        # For this setup, self.tree is built in handle_full_map_response, so assume it matches points_np.

        tree = cKDTree(points_np)
        pairs = tree.query_pairs(min_dist)
        keep_mask = np.ones(points_np.shape[0], dtype=bool)
        for i, j in pairs:
            keep_mask[j] = False  # Remove duplicates

        filtered_points = points_np[keep_mask]
        # self.get_logger().warning(f'Removed {points_np.shape[0] - filtered_points.shape[0]} close points.')
        return filtered_points

    def remove_isolated_points(self, points_np):
        """
        Removes isolated points from a NumPy array of points.

        Args:
            points_np (np.ndarray): Nx3 NumPy array of 3D points.

        Returns:
            np.ndarray: Filtered NumPy array of inlier points.
        """
        if points_np.shape[0] < self.min_neighbors + 1:
            # self.get_logger().warning(
            #     f"Not enough points ({points_np.shape[0]}) to check for isolated points with min_neighbors={self.min_neighbors}. Skipping."
            # )
            return points_np

        # Efficient batch neighbor counting using cKDTree.query_ball_point
        tree = cKDTree(points_np)
        # Use list comprehension for efficiency
        neighbor_counts = np.array([
            len(neigh) - 1  # exclude the point itself
            for neigh in tree.query_ball_point(points_np, self.radius)
        ])
        keep_mask = neighbor_counts >= self.min_neighbors

        filtered_points = points_np[keep_mask]
        # self.get_logger().warning(
        #     f'Removed {points_np.shape[0] - filtered_points.shape[0]} isolated points.'
        # )
        return filtered_points


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()