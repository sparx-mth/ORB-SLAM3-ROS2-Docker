from setuptools import setup

package_name = 'orb_slam3_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/orb_slam3_system.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadav',
    maintainer_email='nadavcherry@gmail.com',
    description='Frontier-based planner for ORB-SLAM3 robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'landmark_publisher_node = orb_slam3_planner.landmark_publisher_node:main',
            'autonomous_explorer_node = orb_slam3_planner.main_node:main',
            'autonomous_explorer_visualizer = orb_slam3_planner.autonomous_explorer_visualizer:main',
        ],
    },
)
