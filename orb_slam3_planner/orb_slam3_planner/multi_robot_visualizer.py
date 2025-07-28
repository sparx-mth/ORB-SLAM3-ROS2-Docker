#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R
import yaml

class ImprovedMultiRobotVisualizer(Node):
    def __init__(self, show_trajectory=True, trajectory_length=200):
        super().__init__('improved_multi_robot_visualizer')

        # Declare and read robot_configs parameter
        self.declare_parameter('robot_configs', '{}')
        robot_configs_yaml = self.get_parameter('robot_configs').value
        self.robot_configs = yaml.safe_load(robot_configs_yaml)

        self.show_trajectory = show_trajectory
        self.trajectory_length = trajectory_length

        # Data storage
        self.robot_poses = {rid: None for rid in self.robot_configs.keys()}
        self.trajectories = {rid: deque(maxlen=trajectory_length) for rid in self.robot_configs.keys()}

        # Transformation matrices
        self.robot_transforms = {}
        for robot_id, config in self.robot_configs.items():
            self.robot_transforms[robot_id] = self.create_transformation_matrix(
                config['position'], config.get('orientation', [0, 0, 0])
            )

        # Merged map from multi_robot_map_merger
        self.merged_map_points = np.empty((0, 4))  # x, y, z, robot_id

        # Visualization state
        self.vis_lock = threading.Lock()
        self.should_reset_view = False
        self.first_pose_received = {rid: False for rid in self.robot_configs.keys()}

        # Visualization options
        self.show_grid = True
        self.show_axes = True
        self.point_size = 2.0

        self.get_logger().info(f'Initializing Improved Multi-Robot Visualizer for robots: {list(self.robot_configs.keys())}')

        # Subscribe to robot poses only
        for robot_id in self.robot_configs.keys():
            # Pose subscription
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/robot_pose_slam',
                self.pose_callback_factory(robot_id),
                10
            )

        # Subscribe ONLY to merged map from multi_robot_map_merger
        self.create_subscription(
            PointCloud2,
            '/merged_map',
            self.merged_map_callback,
            10
        )

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

    def merged_map_callback(self, msg):
        """Receive merged map from multi_robot_map_merger"""
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

    def visualizer_loop(self):
        """Main visualization loop"""
        # Create visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Multi-Robot SLAM Visualization (Merged Map)',
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

        # Merged map point cloud
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

                # Update merged map from multi_robot_map_merger
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

    def get_merged_color(self, robot_id):
        """Get colors for merged map points"""
        base_color = np.array(self.get_robot_color(robot_id))
        return base_color * 0.8

    def print_status(self):
        """Print current status of visualization"""
        status_msg = "\n========== Multi-Robot SLAM Visualization Status =========="

        # Count points per robot in merged map
        robot_point_counts = {}
        for rid in self.robot_configs.keys():
            robot_point_counts[rid] = 0

        if self.merged_map_points.shape[0] > 0:
            for point in self.merged_map_points:
                robot_id = int(point[3])
                if robot_id in robot_point_counts:
                    robot_point_counts[robot_id] += 1

        for robot_id in self.robot_configs.keys():
            pose = self.robot_poses[robot_id]
            traj_len = len(self.trajectories[robot_id])

            if pose is not None:
                status_msg += f"\nRobot_{robot_id}: Pose=[{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}]"
                status_msg += f" | Points in merged map: {robot_point_counts[robot_id]} | Trajectory: {traj_len}"
            else:
                initial_pos = self.robot_configs[robot_id]['position']
                status_msg += f"\nRobot_{robot_id}: At initial position {initial_pos}"

        status_msg += f"\n\nTotal Merged Map Points: {self.merged_map_points.shape[0]}"
        status_msg += "\n" + "=" * 58

        self.get_logger().info(status_msg)


def main(args=None):
    rclpy.init(args=args)
    # Create visualizer
    node = ImprovedMultiRobotVisualizer(
        show_trajectory=True,
        trajectory_length=500
    )

    try:
        node.get_logger().info("Starting simplified multi-robot visualization...")
        node.get_logger().info("Features:")
        node.get_logger().info("- Visualizes ONLY the merged map from map merger")
        node.get_logger().info("- Robot poses and trajectories")
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