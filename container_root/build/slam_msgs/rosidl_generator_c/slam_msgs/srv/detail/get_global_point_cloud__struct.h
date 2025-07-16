// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from slam_msgs:srv/GetGlobalPointCloud.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__STRUCT_H_
#define SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetGlobalPointCloud in the package slam_msgs.
typedef struct slam_msgs__srv__GetGlobalPointCloud_Request
{
  float global_voxel_resolution;
  float local_voxel_resolution;
  /// points above this discarded
  float z_thresh_max;
  bool get_grayscale;
} slam_msgs__srv__GetGlobalPointCloud_Request;

// Struct for a sequence of slam_msgs__srv__GetGlobalPointCloud_Request.
typedef struct slam_msgs__srv__GetGlobalPointCloud_Request__Sequence
{
  slam_msgs__srv__GetGlobalPointCloud_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} slam_msgs__srv__GetGlobalPointCloud_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetGlobalPointCloud in the package slam_msgs.
typedef struct slam_msgs__srv__GetGlobalPointCloud_Response
{
  bool response;
} slam_msgs__srv__GetGlobalPointCloud_Response;

// Struct for a sequence of slam_msgs__srv__GetGlobalPointCloud_Response.
typedef struct slam_msgs__srv__GetGlobalPointCloud_Response__Sequence
{
  slam_msgs__srv__GetGlobalPointCloud_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} slam_msgs__srv__GetGlobalPointCloud_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__STRUCT_H_
