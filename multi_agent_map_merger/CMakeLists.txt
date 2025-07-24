cmake_minimum_required(VERSION 3.10)
project(multi_agent_map_merger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Verbose output for debugging
message(STATUS "Building multi_agent_map_merger")

# Set ORB_SLAM3 path manually if it's not installed as a CMake package
set(ORB_SLAM3_ROOT "/home/orb/ORB_SLAM3")  

# ROS2 & external dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orb_slam3_ros2_wrapper REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)




# Include ORB_SLAM3 and its dependencies manually
include_directories(
  include
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/DBoW2
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/../orb_slam3_ros2_wrapper/include
  ${OPENGL_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
)

# # Optional if you build libORB_SLAM3.so
# link_directories(${ORB_SLAM3_ROOT}/lib)

add_executable(map_merger_node
  src/map_merger_slam_base_node.cpp
  src/map_visualizer.cpp
)

ament_target_dependencies(map_merger_node
  orb_slam3_ros2_wrapper
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  slam_msgs
  OpenCV
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# Link to ORB_SLAM3 shared lib manually
target_link_libraries(map_merger_node
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
)

install(TARGETS map_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
