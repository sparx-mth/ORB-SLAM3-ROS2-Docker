// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from slam_msgs:srv/GetAllLandmarksInMap.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__STRUCT_H_
#define SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetAllLandmarksInMap in the package slam_msgs.
typedef struct slam_msgs__srv__GetAllLandmarksInMap_Request
{
  bool request;
} slam_msgs__srv__GetAllLandmarksInMap_Request;

// Struct for a sequence of slam_msgs__srv__GetAllLandmarksInMap_Request.
typedef struct slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence
{
  slam_msgs__srv__GetAllLandmarksInMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'landmarks'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in srv/GetAllLandmarksInMap in the package slam_msgs.
typedef struct slam_msgs__srv__GetAllLandmarksInMap_Response
{
  sensor_msgs__msg__PointCloud2 landmarks;
} slam_msgs__srv__GetAllLandmarksInMap_Response;

// Struct for a sequence of slam_msgs__srv__GetAllLandmarksInMap_Response.
typedef struct slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence
{
  slam_msgs__srv__GetAllLandmarksInMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__STRUCT_H_
