import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    orb_slam3_launch_file_dir = os.path.join(
        get_package_share_directory('orb_slam3_ros2_wrapper'), 'launch')

    orb_slam3_launch_file_path = os.path.join(
            orb_slam3_launch_file_dir, 'rgbd.launch.py')
            
    # Spawn two ORB-SLAM3 instances
    orb_slam3_launch_description = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                orb_slam3_launch_file_path),
            launch_arguments={"robot_namespace": f"robot_{i}/", 
            		      "robot_x": f"0.{i}",
                              "robot_y": f"0.0"}.items(),
        )
        for i in range(0, 1)
    ]

    return LaunchDescription(
        orb_slam3_launch_description
    )
