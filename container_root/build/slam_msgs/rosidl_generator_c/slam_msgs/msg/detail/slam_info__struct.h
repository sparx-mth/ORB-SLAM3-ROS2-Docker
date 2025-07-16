// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from slam_msgs:msg/SlamInfo.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__MSG__DETAIL__SLAM_INFO__STRUCT_H_
#define SLAM_MSGS__MSG__DETAIL__SLAM_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SlamInfo in the package slam_msgs.
typedef struct slam_msgs__msg__SlamInfo
{
  int32_t num_maps;
  int32_t num_keyframes_in_current_map;
  float tracking_frequency;
} slam_msgs__msg__SlamInfo;

// Struct for a sequence of slam_msgs__msg__SlamInfo.
typedef struct slam_msgs__msg__SlamInfo__Sequence
{
  slam_msgs__msg__SlamInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} slam_msgs__msg__SlamInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SLAM_MSGS__MSG__DETAIL__SLAM_INFO__STRUCT_H_
