from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    agent_names = ['agent_0', 'agent_1']  # can be modified to include more agents as needed
    agent_colors = {
        'agent_0': [1.0, 0.0, 0.0],  # Red
        'agent_1': [0.0, 0.0, 1.0],  # Blue
    }

    refined_color = [0.0, 1.0, 0.0]  # Green for refined map
    return LaunchDescription([
        DeclareLaunchArgument(
            "vocabulary_path",
            default_value="/path/to/ORBvoc.txt",
            description="Path to ORB vocabulary file"
        ),

        Node(
            package="multi_agent_map_merger",
            executable="map_merger_node",
            name="map_merger_node",
            output="screen",
            parameters=[
                {
                    "vocabulary_path": LaunchConfiguration("vocabulary_path"),
                    "agent_colors": agent_colors,
                    "refined_color": refined_color
                }
            ],
            arguments=agent_names  # Pass agent names as arguments
        )
    ])
