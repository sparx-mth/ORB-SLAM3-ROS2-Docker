from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch description for the full multi-robot autonomous exploration system.

    This launch file initializes:
    - A centralized occupancy grid mapper that processes merged 3D point clouds.
    - A global map merger that aligns and fuses 3D landmarks from multiple robots.
    - A 2D visualizer using OpenCV to display grid map, paths, goals, and robot locations.
    - A 3D visualizer using Open3D to show merged point cloud and robot trajectories.
    - One autonomous exploration node per robot, handling planning and motion.
    - One landmark publisher node per robot, requesting and publishing raw SLAM landmarks.

    Each node receives the `robot_configs.yaml` which defines robot IDs, positions, and transforms.
    """

    # Load robot configuration YAML
    config_file = os.path.join(
        get_package_share_directory('orb_slam3_planner'),
        'config',
        'robot_configs.yaml'
    )
    with open(config_file, 'r') as f:
        robot_configs = yaml.safe_load(f)
    robot_configs_yaml = yaml.dump(robot_configs)  # pass as parameter to nodes

    nodes = []

    # === 1. Occupancy Grid Builder ===
    # Converts merged 3D point cloud into 2D occupancy grid.
    # Publishes /occupancy_grid and robot grid positions (PoseArray).
    nodes.append(
        Node(
            package='orb_slam3_planner',
            executable='multi_robot_map_builder',
            name='multi_robot_map_builder',
            output='screen',
            parameters=[{'robot_configs': robot_configs_yaml}]
        )
    )

    # === 2. Landmark-Based Map Merger ===
    # Merges landmark clouds from each robot into a single fused global map.
    # Outputs /merged_map (PointCloud2) and filters noise.
    nodes.append(
        Node(
            package='orb_slam3_planner',
            executable='multi_robot_map_merger',
            name='multi_robot_map_merger',
            output='screen',
            parameters=[{'robot_configs': robot_configs_yaml}]
        )
    )

    # === 3. 2D Grid Map Visualizer ===
    # Displays the occupancy grid, robot positions, paths, and goals in OpenCV window.
    nodes.append(
        Node(
            package='orb_slam3_planner',
            executable='multi_robot_visualizer_2d',
            name='multi_robot_visualizer_2d',
            output='screen',
            parameters=[{'robot_configs': robot_configs_yaml}]
        )
    )

    # === 4. 3D Point Cloud Visualizer ===
    # Uses Open3D to visualize merged landmarks and robot trajectories.
    nodes.append(
        Node(
            package='orb_slam3_planner',
            executable='multi_robot_visualizer',
            name='multi_robot_visualizer',
            output='screen',
            parameters=[{'robot_configs': robot_configs_yaml}]
        )
    )

    # === 5. Per-Robot Nodes ===
    for robot_id in robot_configs.keys():
        namespace = f'robot_{robot_id}'

        # --- 5a. Autonomous Explorer ---
        # Handles planning (frontier detection) and navigation using A*.
        # Receives map, tracks pose, and publishes cmd_vel and goals.
        nodes.append(
            Node(
                package='orb_slam3_planner',
                executable='autonomous_explorer_node',
                name=f'autonomous_explorer_{robot_id}',
                namespace='',
                parameters=[{
                    'robot_namespace': namespace,
                    'robot_configs': robot_configs_yaml
                }],
                output='screen',
            )
        )

        # --- 5b. Landmark Publisher ---
        # Periodically requests landmarks from the robot's SLAM service
        # and publishes them to /robot_X/orb_slam3/landmarks_raw.
        nodes.append(
            Node(
                package='orb_slam3_planner',
                executable='landmark_publisher_node',
                name=f'landmark_publisher_{robot_id}',
                namespace='',
                parameters=[{
                    'robot_namespace': namespace,
                    'robot_id': robot_id
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0
            )
        )

    return LaunchDescription(nodes)
