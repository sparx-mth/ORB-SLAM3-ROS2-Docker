from setuptools import setup

package_name = 'orb_slam3_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/orb_slam3_system.launch.py']),
        ('share/' + package_name + '/config', ['config/robot_configs.yaml']),
    ],

    install_requires=[
        'setuptools',
        'open3d',
        'numpy==1.26.4',
    ],
    zip_safe=True,
    maintainer='nadav',
    maintainer_email='nadavcherry@gmail.com',
    description='Frontier-based planner for ORB-SLAM3 robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'landmark_publisher_node = orb_slam3_planner.landmark_publisher_node:main',
            'autonomous_explorer_node = orb_slam3_planner.main_node:main',
            'multi_robot_visualizer = orb_slam3_planner.multi_robot_visualizer:main',
            'multi_robot_map_builder = orb_slam3_planner.multi_robot_map_builder:main',
            'multi_robot_visualizer_2d = orb_slam3_planner.multi_robot_visualizer_2d:main',
            'multi_robot_map_merger = orb_slam3_planner.multi_robot_map_merger:main',

        ],
    },
)