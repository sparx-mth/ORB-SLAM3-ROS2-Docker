#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from slam_msgs.srv import GetAllLandmarksInMap
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import threading
import time
from collections import deque
import copy
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree


class ImprovedMultiRobotVisualizer(Node):
    def __init__(self, robot_configs, show_trajectory=True, trajectory_length=200):
        super().__init__('improved_multi_robot_visualizer')

        self.robot_configs = robot_configs
        self.show_trajectory = show_trajectory
        self.trajectory_length = trajectory_length

        # Data storage
        self.robot_poses = {rid: None for rid in robot_configs.keys()}
        self.robot_landmarks = {rid: np.empty((0, 3)) for rid in robot_configs.keys()}
        self.raw_landmarks = {rid: np.empty((0, 3)) for rid in robot_configs.keys()}
        self.trajectories = {rid: deque(maxlen=trajectory_length) for rid in robot_configs.keys()}

        # Transformation matrices
        self.robot_transforms = {}
        for robot_id, config in robot_configs.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config.get('orientation', [0, 0, 0])
            )

        # Merged map
        self.merged_map_points = np.empty((0, 4))  # x, y, z, robot_id
        self.merged_cloud_data = None

        # Visualization state
        self.vis_lock = threading.Lock()
        self.should_reset_view = False
        self.first_pose_received = {rid: False for rid in robot_configs.keys()}

        # ICP alignment parameters
        self.enable_icp = True
        self.icp_fitness_threshold = 0.3
        self.icp_iterations = 50

        # Visualization options
        self.show_grid = True
        self.show_axes = True
        self.point_size = 2.0

        self.get_logger().info(f'Initializing Improved Multi-Robot Visualizer for robots: {list(robot_configs.keys())}')

        # Subscribe to topics
        for robot_id in robot_configs.keys():
            # Pose subscription
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/robot_pose_slam',
                self.pose_callback_factory(robot_id),
                10
            )

            # Real-time landmarks
            self.create_subscription(
                PointCloud2,
                f'/robot_{robot_id}/orb_slam3/landmarks',
                self.landmarks_callback_factory(robot_id),
                10
            )

            # Raw landmarks
            self.create_subscription(
                PointCloud2,
                f'/robot_{robot_id}/orb_slam3/landmarks_raw',
                self.raw_landmarks_callback_factory(robot_id),
                10
            )

        # Subscribe to merged map
        self.create_subscription(
            PointCloud2,
            '/merged_map',
            self.merged_map_callback,
            10
        )

        # Service clients
        self.landmark_clients = {}
        for robot_id in robot_configs.keys():
            service_name = f'/robot_{robot_id}/orb_slam3/get_all_landmarks_in_map'
            self.landmark_clients[robot_id] = self.create_client(GetAllLandmarksInMap, service_name)

        # Timers
        self.map_request_timer = self.create_timer(2.0, self.request_full_maps)
        self.alignment_timer = self.create_timer(5.0, self.perform_map_alignment)

        # Launch visualizer
        self.vis_thread = threading.Thread(target=self.visualizer_loop, daemon=True)
        self.vis_thread.start()
        self.get_logger().info('Open3D visualizer thread started.')

    def create_transformation_matrix(self, position, orientation):
        """Create 4x4 transformation matrix"""
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def pose_callback_factory(self, robot_id):
        def callback(msg):
            with self.vis_lock:
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

                # Add to trajectory
                self.trajectories[robot_id].append(global_pose[:3].copy())

                if not self.first_pose_received[robot_id]:
                    self.first_pose_received[robot_id] = True
                    self.should_reset_view = True

        return callback

    def landmarks_callback_factory(self, robot_id):
        def callback(msg):
            try:
                points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                if points:
                    with self.vis_lock:
                        # Transform points to global frame
                        local_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
                        global_points = self.transform_points_to_global(local_points, robot_id)
                        self.robot_landmarks[robot_id] = global_points
            except Exception as e:
                self.get_logger().error(f'Error processing landmarks for robot_{robot_id}: {e}')

        return callback

    def raw_landmarks_callback_factory(self, robot_id):
        def callback(msg):
            try:
                points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                if points:
                    with self.vis_lock:
                        # Transform points to global frame
                        local_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
                        global_points = self.transform_points_to_global(local_points, robot_id)
                        self.raw_landmarks[robot_id] = global_points
            except Exception as e:
                self.get_logger().error(f'Error processing raw landmarks for robot_{robot_id}: {e}')

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

    def merged_map_callback(self, msg):
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
            if points:
                with self.vis_lock:
                    parsed_points = []
                    for point in points:
                        x, y, z, rgb = point[0], point[1], point[2], point[3]

                        # Decode robot ID from RGB
                        rgb_int = int(rgb)
                        r = (rgb_int >> 16) & 0xFF
                        g = (rgb_int >> 8) & 0xFF
                        b = rgb_int & 0xFF

                        # Map colors to robot IDs
                        if b == 255 and r == 0 and g == 0:
                            robot_id = 0
                        elif g == 255 and r == 0 and b == 0:
                            robot_id = 1
                        elif r == 255 and g == 0 and b == 0:
                            robot_id = 2
                        else:
                            robot_id = -1

                        parsed_points.append([x, y, z, robot_id])

                    self.merged_map_points = np.array(parsed_points, dtype=np.float64)
                    self.get_logger().info(f'Merged map updated: {len(points)} points')
        except Exception as e:
            self.get_logger().error(f'Error processing merged map: {e}')

    def request_full_maps(self):
        """Request full landmark maps from each robot"""
        for robot_id, client in self.landmark_clients.items():
            if client.service_is_ready():
                request = GetAllLandmarksInMap.Request()
                request.request = True
                future = client.call_async(request)
                future.add_done_callback(lambda f, rid=robot_id: self.handle_full_map_response(f, rid))

    def handle_full_map_response(self, future, robot_id):
        try:
            response = future.result()
            if response:
                points = list(pc2.read_points(response.landmarks, field_names=("x", "y", "z"), skip_nans=True))
                if points:
                    with self.vis_lock:
                        local_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
                        global_points = self.transform_points_to_global(local_points, robot_id)
                        self.raw_landmarks[robot_id] = global_points
                    self.get_logger().info(f'Full map from robot_{robot_id}: {len(points)} points')
        except Exception as e:
            self.get_logger().error(f'Error getting full map from robot_{robot_id}: {e}')

    def perform_map_alignment(self):
        """Perform ICP alignment between robot maps if enabled"""
        if not self.enable_icp:
            return

        with self.vis_lock:
            # Skip if we don't have enough data
            if len(self.raw_landmarks) < 2:
                return

            # Find robots with sufficient landmarks
            valid_robots = []
            for rid, landmarks in self.raw_landmarks.items():
                if len(landmarks) > 100:  # Minimum points for ICP
                    valid_robots.append(rid)

            if len(valid_robots) < 2:
                return

            # Perform pairwise ICP alignment
            # For now, just log that we could do it
            self.get_logger().info(f'Could perform ICP alignment between robots: {valid_robots}')

    def visualizer_loop(self):
        """Main visualization loop"""
        # Create visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Improved Multi-Robot SLAM Visualization',
            width=1920,
            height=1080
        )

        # Set rendering options
        render_option = vis.get_render_option()
        render_option.point_size = self.point_size
        render_option.background_color = np.array([0.1, 0.1, 0.1])
        render_option.show_coordinate_frame = self.show_axes

        # Initialize geometries
        robot_spheres = {}
        trajectory_lines = {}
        landmark_clouds = {}
        raw_landmark_clouds = {}

        # Merged map
        merged_cloud = o3d.geometry.PointCloud()
        vis.add_geometry(merged_cloud)

        # Create geometries for each robot
        for robot_id in self.robot_configs.keys():
            # Robot sphere
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
            sphere.paint_uniform_color(self.get_robot_color(robot_id))
            sphere.compute_vertex_normals()

            # Initial position
            config = self.robot_configs[robot_id]
            initial_pos = config['position']
            sphere.translate(initial_pos)

            robot_spheres[robot_id] = sphere
            vis.add_geometry(sphere)

            # Trajectory
            if self.show_trajectory:
                trajectory_lines[robot_id] = o3d.geometry.LineSet()
                vis.add_geometry(trajectory_lines[robot_id])

            # Point clouds
            landmark_clouds[robot_id] = o3d.geometry.PointCloud()
            vis.add_geometry(landmark_clouds[robot_id])

            raw_landmark_clouds[robot_id] = o3d.geometry.PointCloud()
            vis.add_geometry(raw_landmark_clouds[robot_id])

        # Add coordinate frame at origin
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])
        vis.add_geometry(coord_frame)

        # Add grid if enabled
        if self.show_grid:
            grid_size = 20
            grid_points = []
            grid_lines = []
            point_idx = 0

            # Create grid points
            for x in range(-grid_size, grid_size + 1):
                for y in range(-grid_size, grid_size + 1):
                    grid_points.append([x, y, 0])

            # Create grid lines
            for i in range(2 * grid_size + 1):
                # X-direction lines
                for j in range(2 * grid_size):
                    idx = i * (2 * grid_size + 1) + j
                    grid_lines.append([idx, idx + 1])

                # Y-direction lines
                if i < 2 * grid_size:
                    for j in range(2 * grid_size + 1):
                        idx1 = i * (2 * grid_size + 1) + j
                        idx2 = (i + 1) * (2 * grid_size + 1) + j
                        grid_lines.append([idx1, idx2])

            grid = o3d.geometry.LineSet()
            grid.points = o3d.utility.Vector3dVector(grid_points)
            grid.lines = o3d.utility.Vector2iVector(grid_lines)
            grid.paint_uniform_color([0.3, 0.3, 0.3])
            vis.add_geometry(grid)

        # Main visualization loop
        self.get_logger().info("Starting visualization loop...")
        frame_count = 0

        while True:
            time.sleep(0.033)  # ~30 FPS
            frame_count += 1

            with self.vis_lock:
                # Update each robot's visualization
                for robot_id in self.robot_configs.keys():

                    # Update robot position
                    if self.robot_poses[robot_id] is not None:
                        try:
                            sphere = robot_spheres[robot_id]
                            current_center = np.asarray(sphere.get_center())
                            target_position = self.robot_poses[robot_id]
                            translation_vector = target_position - current_center

                            sphere.translate(translation_vector, relative=True)
                            vis.update_geometry(sphere)
                        except Exception as e:
                            self.get_logger().error(f'Error updating robot pose for robot_{robot_id}: {e}')

                    # Update trajectory
                    if self.show_trajectory and len(self.trajectories[robot_id]) > 1:
                        try:
                            trajectory = trajectory_lines[robot_id]
                            trajectory_points = list(self.trajectories[robot_id])

                            points = o3d.utility.Vector3dVector(trajectory_points)
                            lines = [[i, i + 1] for i in range(len(trajectory_points) - 1)]

                            trajectory.points = points
                            trajectory.lines = o3d.utility.Vector2iVector(lines)

                            # Gradient color for trajectory
                            colors = []
                            for i in range(len(lines)):
                                t = i / max(1, len(lines) - 1)
                                color = np.array(self.get_robot_color(robot_id)) * (0.3 + 0.7 * t)
                                colors.append(color)
                            trajectory.colors = o3d.utility.Vector3dVector(colors)

                            vis.update_geometry(trajectory)
                        except Exception as e:
                            self.get_logger().error(f'Error updating trajectory for robot_{robot_id}: {e}')

                    # Update raw landmarks (full map)
                    if self.raw_landmarks[robot_id].shape[0] > 0:
                        try:
                            raw_cloud = raw_landmark_clouds[robot_id]
                            raw_cloud.points = o3d.utility.Vector3dVector(self.raw_landmarks[robot_id])
                            raw_cloud.paint_uniform_color(self.get_landmark_color(robot_id))
                            vis.update_geometry(raw_cloud)
                        except Exception as e:
                            self.get_logger().error(f'Error updating raw landmarks for robot_{robot_id}: {e}')

                    # Update real-time landmarks
                    if self.robot_landmarks[robot_id].shape[0] > 0:
                        try:
                            landmark_cloud = landmark_clouds[robot_id]
                            landmark_cloud.points = o3d.utility.Vector3dVector(self.robot_landmarks[robot_id])
                            light_color = np.array(self.get_robot_color(robot_id)) * 0.5 + 0.5
                            landmark_cloud.paint_uniform_color(light_color)
                            vis.update_geometry(landmark_cloud)
                        except Exception as e:
                            self.get_logger().error(f'Error updating landmarks for robot_{robot_id}: {e}')

                # Update merged map
                if self.merged_map_points.shape[0] > 0:
                    try:
                        merged_cloud.points = o3d.utility.Vector3dVector(self.merged_map_points[:, :3])

                        # Color based on robot ID
                        colors = []
                        for point in self.merged_map_points:
                            robot_id = int(point[3])
                            if robot_id >= 0:
                                colors.append(self.get_merged_color(robot_id))
                            else:
                                colors.append([0.5, 0.5, 0.5])  # Gray for unknown

                        merged_cloud.colors = o3d.utility.Vector3dVector(colors)
                        vis.update_geometry(merged_cloud)
                    except Exception as e:
                        self.get_logger().error(f'Error updating merged map: {e}')

                # Reset view if needed
                if self.should_reset_view:
                    vis.reset_view_point(True)
                    self.should_reset_view = False

                    # Set a better initial viewpoint
                    ctr = vis.get_view_control()
                    ctr.set_lookat([0, 0, 0])
                    ctr.set_up([0, 0, 1])
                    ctr.set_front([1, 1, -1])
                    ctr.set_zoom(0.5)

            # Update visualizer
            vis.poll_events()
            vis.update_renderer()

            # Print status periodically
            if frame_count % 150 == 0:  # Every 5 seconds at 30 FPS
                self.print_status()

    def get_robot_color(self, robot_id):
        """Get distinct colors for each robot"""
        colors = {
            0: [0.2, 0.4, 1.0],  # Blue
            1: [0.2, 1.0, 0.2],  # Green
            2: [1.0, 0.2, 0.2],  # Red
            3: [1.0, 1.0, 0.2],  # Yellow
            4: [1.0, 0.2, 1.0],  # Magenta
            5: [0.2, 1.0, 1.0],  # Cyan
        }
        return colors.get(robot_id, [0.7, 0.7, 0.7])

    def get_landmark_color(self, robot_id):
        """Get colors for landmark clouds"""
        base_color = np.array(self.get_robot_color(robot_id))
        return base_color * 0.8

    def get_trajectory_color(self, robot_id):
        """Get trajectory colors"""
        base_color = np.array(self.get_robot_color(robot_id))
        return base_color * 0.6

    def get_merged_color(self, robot_id):
        """Get colors for merged map points"""
        base_color = np.array(self.get_robot_color(robot_id))
        return base_color * 0.6

    def print_status(self):
        """Print current status of all robots"""
        status_msg = "\n========== Multi-Robot SLAM Status =========="

        total_raw_points = 0
        total_rt_points = 0

        for robot_id in self.robot_configs.keys():
            pose = self.robot_poses[robot_id]
            landmarks_count = self.robot_landmarks[robot_id].shape[0]
            raw_count = self.raw_landmarks[robot_id].shape[0]
            traj_len = len(self.trajectories[robot_id])

            total_raw_points += raw_count
            total_rt_points += landmarks_count

            if pose is not None:
                status_msg += f"\nRobot_{robot_id}: Pose=[{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}]"
                status_msg += f" | RT_Points={landmarks_count} | Map_Points={raw_count} | Trajectory={traj_len}"
            else:
                initial_pos = self.robot_configs[robot_id]['position']
                status_msg += f"\nRobot_{robot_id}: At initial position {initial_pos} | Map_Points={raw_count}"

        status_msg += f"\n\nMerged Map: {self.merged_map_points.shape[0]} points"
        status_msg += f"\nTotal Points: {total_raw_points} (Raw) | {total_rt_points} (Real-time)"
        status_msg += "\n" + "=" * 45

        self.get_logger().info(status_msg)


def main(args=None):
    rclpy.init(args=args)

    # Robot configurations with positions and orientations
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.0], 'orientation': [0.0, 0.0, 0.0]},
        1: {'position': [-1.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]},
        2: {'position': [5.0, 5.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}
    }

    # Create visualizer
    node = ImprovedMultiRobotVisualizer(
        robot_configs=robot_configs,
        show_trajectory=True,
        trajectory_length=500
    )

    try:
        node.get_logger().info("Starting improved multi-robot visualization...")
        node.get_logger().info("Features:")
        node.get_logger().info("- Proper coordinate transformations")
        node.get_logger().info("- Real-time and full map visualization")
        node.get_logger().info("- Robot trajectories")
        node.get_logger().info("- Merged map with duplicate removal")
        node.get_logger().info("- Grid and coordinate frames")
        node.get_logger().info("\nPress Ctrl+C to exit")

        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()