cmake_minimum_required(VERSION 3.10)
project(multi_agent_map_merger)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_msgs REQUIRED)  # You should create this .msg/.srv package
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ORB-SLAM3 dependencies (assuming you're linking to it manually)
# Include ORB-SLAM3 headers and libs (adjust these as needed)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Sources
add_executable(map_merger_slam_node
  src/map_merger_slam_base_node.cpp
  src/main_map_merger.cpp
)

ament_target_dependencies(map_merger_slam_node
  rclcpp
  std_msgs
  slam_msgs
  OpenCV
)

install(TARGETS
  map_merger_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
