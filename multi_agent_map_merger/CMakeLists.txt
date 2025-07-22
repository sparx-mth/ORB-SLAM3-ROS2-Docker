cmake_minimum_required(VERSION 3.10)
project(multi_agent_map_merger)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ORB-SLAM3 includes
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/include
  ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2
  ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o
)

# Declare source files
set(SOURCES
  src/map_merger_slam_base_node.cpp
  src/map_visualizer.cpp
)

add_executable(map_merger_node ${SOURCES})
ament_target_dependencies(map_merger_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  slam_msgs
  OpenCV
  Eigen3
  tf2
  tf2_ros
)

target_include_directories(map_merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS map_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
