from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for multi-robot exploration system.

    This launches:
    1. One shared map builder (multi_robot_map_builder.py)
    2. One map merger (multi_robot_map_merger.py)
    3. Three autonomous explorers (main_node.py) - one for each robot
    4. Three visualizers (autonomous_explorer_visualizer.py) - one for each robot
    5. Three landmark publishers (landmark_publisher_node.py) - one for each robot
    """

    # Robot configurations must match those in multi_robot_map_builder.py
    robot_configs = {
        0: {'position': [-5.0, -7.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 1: {'position': [-1.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0]},
        # 2: {'position': [5.0, 5.0, 0.5], 'orientation': [0.0, 0.0, 0.0]}
    }

    nodes = []

    # 1. Launch shared map builder (only one instance)
    nodes.append(
        Node(
            package='orb_slam3_planner',
            executable='multi_robot_map_builder',
            name='multi_robot_map_builder',
            output='screen'
        )
    )

    # # 2. Launch map merger (only one instance)
    # nodes.append(
    #     Node(
    #         package='orb_slam3_planner',
    #         executable='multi_robot_map_merger',
    #         name='multi_robot_map_merger',
    #         output='screen'
    #     )
    # )

    nodes.append(
        Node(
            package='orb_slam3_planner',
            executable='multi_robot_visualizer_2d',
            name='multi_robot_visualizer_2d',
            output='screen'
        )
    )
    #
    # nodes.append(
    #     Node(
    #         package='orb_slam3_planner',
    #         executable='multi_robot_visualizer',
    #         name='multi_robot_visualizer',
    #         output='screen'
    #     )
    # )

    # 3. Launch individual robot nodes
    for robot_id in robot_configs.keys():
        namespace = f'robot_{robot_id}'

        # Main autonomous explorer node
        nodes.append(
            Node(
                package='orb_slam3_planner',
                executable='autonomous_explorer_node',
                name=f'autonomous_explorer_{robot_id}',
                namespace='',
                parameters=[{
                    'robot_namespace': namespace
                }],
                output='screen',
            )
        )

        # Landmark publisher for each robot
        nodes.append(
            Node(
                package='orb_slam3_planner',
                executable='landmark_publisher_node',
                name=f'landmark_publisher_{robot_id}',
                namespace='',  # Empty namespace, use parameter instead
                parameters=[{
                    'robot_namespace': namespace,
                    'robot_id': robot_id
                }],
                output='screen',
                respawn=True,  # Restart if it crashes
                respawn_delay=2.0
            )
        )
    return LaunchDescription(nodes)