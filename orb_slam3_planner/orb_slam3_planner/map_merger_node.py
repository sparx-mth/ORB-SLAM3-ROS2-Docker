#!/usr/bin/env python3

import struct
import rclpy
from rclpy.node import Node
from slam_msgs.srv import GetAllLandmarksInMap
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
import time
import numpy as np
import std_msgs.msg
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R


class ImprovedMapMergerNode(Node):
    def __init__(self):
        super().__init__('improved_map_merger_node')

        # Robot configurations with initial poses (x, y, z, roll, pitch, yaw)
        self.robots = {
            0: {'position': [-5.0, -7.0, 1.0], 'orientation': [0.0, 0.0, 0.0]},  # Blue
            1: {'position': [-1.0, 0.0, 1.65], 'orientation': [0.0, 0.0, 0.0]},  # Green
            2: {'position': [5.0, 5.0, 1.65], 'orientation': [0.0, 0.0, 0.0]}  # Red
        }

        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store transformation matrices for each robot
        self.robot_transforms = {}
        for robot_id, config in self.robots.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config['orientation']
            )

        # Create service clients for each robot
        self.agents_clients = {}
        for robot_id in self.robots:
            service_name = f'/robot_{robot_id}/orb_slam3/get_all_landmarks_in_map'
            self.agents_clients[robot_id] = self.create_client(GetAllLandmarksInMap, service_name)

        # Wait for services
        for robot_id, client in self.agents_clients.items():
            while not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().info(f'Service for robot_{robot_id} not available, waiting...')

        # Publishers
        self.map_publisher = self.create_publisher(PointCloud2, '/merged_map', 10)
        self.debug_publisher = self.create_publisher(PointCloud2, '/debug_transformed_points', 10)

        # Data storage
        self.merged_map = []
        self.robot_local_maps = {robot_id: [] for robot_id in self.robots}
        self.last_landmarks = {robot_id: [] for robot_id in self.robots}
        self.map_update_count = {robot_id: 0 for robot_id in self.robots}

        # Subscribe to robot poses for dynamic transform updates
        for robot_id in self.robots:
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/orb_slam3/camera_pose',
                self.create_pose_callback(robot_id),
                10
            )

        # Parameters for map merging
        self.distance_threshold = 0.05  # 5cm threshold for duplicate removal
        self.change_threshold = 0.2  # Threshold for detecting significant changes

        self.get_logger().info('Improved Map Merger Node started.')

        # Timer for periodic map requests
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Timer for publishing transforms
        self.transform_timer = self.create_timer(0.1, self.publish_transforms)

    def create_transformation_matrix(self, position, orientation):
        """Create a 4x4 transformation matrix from position and orientation (RPY)"""
        # Create rotation matrix from roll, pitch, yaw
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        # Create 4x4 transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def create_pose_callback(self, robot_id):
        """Create a callback for robot pose updates"""

        def callback(msg):
            # Update the robot's transformation based on its current pose
            # This helps refine the transformation if robots are moving
            pass  # For now, we'll use fixed transformations

        return callback

    def publish_transforms(self):
        """Publish TF transforms for visualization"""
        for robot_id, config in self.robots.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "world"
            transform.child_frame_id = f"robot_{robot_id}_base"

            # Set translation
            transform.transform.translation.x = config['position'][0]
            transform.transform.translation.y = config['position'][1]
            transform.transform.translation.z = config['position'][2]

            # Set rotation
            rotation = R.from_euler('xyz', config['orientation'])
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(transform)

    def timer_callback(self):
        """Request map data from all robots"""
        for robot_id in self.robots:
            self.get_all_landmarks(robot_id)

    def get_all_landmarks(self, robot_id):
        """Request landmarks from a specific robot"""
        request = GetAllLandmarksInMap.Request()
        request.request = True

        future = self.agents_clients[robot_id].call_async(request)
        future.add_done_callback(lambda future, robot_id=robot_id: self.handle_landmark_response(future, robot_id))

    def handle_landmark_response(self, future, robot_id):
        """Process landmark response and merge into global map"""
        try:
            response = future.result()

            if response and response.landmarks.data:
                # Convert PointCloud2 to numpy array
                points = list(pc2.read_points(response.landmarks, field_names=("x", "y", "z"), skip_nans=True))

                # Filter invalid points
                valid_points = self.filter_invalid_points(points)

                if not valid_points:
                    return

                self.get_logger().info(f'Received {len(valid_points)} valid landmarks from robot_{robot_id}')

                # Check for significant changes
                if self.is_significant_change(robot_id, valid_points):
                    # Transform points to global frame
                    transformed_points = self.transform_landmarks_to_global(valid_points, robot_id)

                    # Update local map for this robot
                    self.robot_local_maps[robot_id] = transformed_points

                    # Merge all robot maps
                    self.merge_all_maps()

                    # Update last landmarks
                    self.last_landmarks[robot_id] = valid_points
                    self.map_update_count[robot_id] += 1

                    # Publish debug visualization
                    self.publish_debug_clouds(robot_id, transformed_points)

        except Exception as e:
            self.get_logger().error(f'Service call failed for robot_{robot_id}: {str(e)}')

    def filter_invalid_points(self, points):
        """Filter out invalid points (NaN or infinite values)"""
        valid_points = []
        for point in points:
            if len(point) >= 3:
                x, y, z = point[0], point[1], point[2]
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or
                        np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    valid_points.append([x, y, z])
        return valid_points

    def is_significant_change(self, robot_id, new_points):
        """Check if the map has changed significantly"""
        if len(self.last_landmarks[robot_id]) < 2:
            return True

        # Sample points for comparison (don't compare all points for efficiency)
        sample_size = min(100, len(new_points), len(self.last_landmarks[robot_id]))

        if sample_size < 10:
            return True

        # Random sampling
        new_sample_indices = np.random.choice(len(new_points), sample_size, replace=False)
        old_sample_indices = np.random.choice(len(self.last_landmarks[robot_id]), sample_size, replace=False)

        # Calculate average change
        total_change = 0
        for i in range(sample_size):
            new_pt = new_points[new_sample_indices[i]]
            old_pt = self.last_landmarks[robot_id][old_sample_indices[i]]
            distance = np.linalg.norm(np.array(new_pt) - np.array(old_pt))
            total_change += distance

        average_change = total_change / sample_size
        return average_change > self.change_threshold

    def transform_landmarks_to_global(self, local_points, robot_id):
        """Transform points from robot's local frame to global frame"""
        transform_matrix = self.robot_transforms[robot_id]
        transformed_points = []

        for point in local_points:
            # Convert to homogeneous coordinates
            local_point_homo = np.array([point[0], point[1], point[2], 1.0])

            # Apply transformation
            global_point_homo = transform_matrix @ local_point_homo

            # Extract 3D coordinates and add robot ID
            transformed_points.append([
                global_point_homo[0],
                global_point_homo[1],
                global_point_homo[2],
                robot_id
            ])

        return transformed_points

    def merge_all_maps(self):
        """Merge maps from all robots with duplicate removal"""
        # Collect all points from all robots
        all_points = []
        for robot_id, points in self.robot_local_maps.items():
            all_points.extend(points)

        if not all_points:
            return

        # Remove duplicates using spatial hashing
        self.merged_map = self.remove_duplicates(all_points)

        # Publish merged map
        self.publish_merged_map()

        self.get_logger().info(f'Merged map: {len(self.merged_map)} unique points from {len(all_points)} total')

    def remove_duplicates(self, points):
        """Remove duplicate points using spatial hashing"""
        if not points:
            return []

        # Create spatial hash grid
        grid_resolution = self.distance_threshold
        point_dict = {}

        for point in points:
            # Calculate grid cell
            grid_x = int(point[0] / grid_resolution)
            grid_y = int(point[1] / grid_resolution)
            grid_z = int(point[2] / grid_resolution)
            grid_key = (grid_x, grid_y, grid_z)

            # Check if there's already a point in this cell or neighboring cells
            found_duplicate = False
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        neighbor_key = (grid_x + dx, grid_y + dy, grid_z + dz)
                        if neighbor_key in point_dict:
                            existing_point = point_dict[neighbor_key]
                            distance = np.linalg.norm(
                                np.array(point[:3]) - np.array(existing_point[:3])
                            )
                            if distance < self.distance_threshold:
                                found_duplicate = True
                                break
                    if found_duplicate:
                        break
                if found_duplicate:
                    break

            if not found_duplicate:
                point_dict[grid_key] = point

        return list(point_dict.values())

    def publish_merged_map(self):
        """Publish the merged point cloud"""
        if not self.merged_map:
            return

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"

        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(self.merged_map)

        # Define fields
        pc2_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
        ]
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width

        # Robot colors
        colors = {
            0: [0, 0, 255],  # Blue
            1: [0, 255, 0],  # Green
            2: [255, 0, 0],  # Red
        }

        # Pack point data
        points_data = bytearray()
        for point in self.merged_map:
            x, y, z, robot_id = point[0], point[1], point[2], int(point[3])

            # Get color for robot
            rgb = colors.get(robot_id, [128, 128, 128])
            rgb_value = (0xFF << 24) | (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]

            # Pack data
            packed_point = struct.pack('<fffI', x, y, z, rgb_value)
            points_data.extend(packed_point)

        pc2_msg.data = bytes(points_data)
        self.map_publisher.publish(pc2_msg)

    def publish_debug_clouds(self, robot_id, transformed_points):
        """Publish debug point clouds for visualization"""
        if not transformed_points:
            return

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"

        # Create point cloud from transformed points
        points = [[p[0], p[1], p[2]] for p in transformed_points]

        # Create PointCloud2
        pc2_msg = pc2.create_cloud_xyz32(header, points)
        self.debug_publisher.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedMapMergerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()