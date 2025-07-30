# Multi-Robot Autonomous Exploration System

This ROS2-based system performs autonomous exploration and SLAM using multiple robots with ORB-SLAM3. It includes shared mapping, distributed planning, and landmark-based map merging.

## Project Structure

| File / Module                        | Description |
|-------------------------------------|-------------|
| `main_node.py`                      | Central node per robot. Handles planning, map reading, movement, and coordination. |
| `planner_module.py`                 | Frontier-based path planner with multi-robot coordination. |
| `drone_controller_module.py`        | Controls robot motion (waypoint following, recovery, obstacle avoidance). |
| `multi_robot_map_builder.py`       | Merges multiple robots' landmarks into a shared 2D occupancy grid. |
| `multi_robot_map_merger.py`        | Merges 3D landmark maps into one colored point cloud. |
| `landmark_publisher_node.py`       | Requests landmarks from ORB-SLAM3 and publishes them for map merging. |
| `multi_robot_visualizer.py`        | 3D Open3D visualizer for robots' trajectories and merged map. |
| `multi_robot_visualizer_2d.py`     | Real-time 2D OpenCV visualizer of occupancy map and robot positions. |
| `launch/orb_slam3_system.launch.py`| Launch file for all robots, map builder, merger, and visualizers. |
| `setup.py`                          | Package install and entry points definition. |

## How to Run

### 1. Build the ROS2 Workspace in the docker

```bash
colcon build --symlink-install
source install/setup.bash
```

```bash
ros2 launch orb_slam3_planner orb_slam3_system.launch.py
```

## Optional Modifications

### Disable the Planner (Control Loop)

If you want to run the system **without the planner**, for example just to test mapping or visualization, comment out the control loop timer in `main_node.py`:

```python
# self.create_timer(0.5, self.control_loop)
```

### Remove Specific Nodes from Execution

To completely disable the launch of a specific node (e.g., the 2D visualizer), you must update **both** the launch file and the package setup file:

**In the launch file (`orb_slam3_system.launch.py`)**, comment out the node:

```python
# nodes.append(
#     Node(
#         package='orb_slam3_planner',
#         executable='multi_robot_visualizer_2d',
#         name='multi_robot_visualizer_2d',
#         output='screen'
#     )
# )
```
In setup.py(`setup.py`), comment out or remove the relevant console_scripts entry:
```python
# 'multi_robot_visualizer_2d = orb_slam3_planner.multi_robot_visualizer_2d:main',
```