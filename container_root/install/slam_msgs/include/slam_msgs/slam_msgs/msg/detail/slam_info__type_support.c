// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from slam_msgs:msg/SlamInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "slam_msgs/msg/detail/slam_info__rosidl_typesupport_introspection_c.h"
#include "slam_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "slam_msgs/msg/detail/slam_info__functions.h"
#include "slam_msgs/msg/detail/slam_info__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  slam_msgs__msg__SlamInfo__init(message_memory);
}

void slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_fini_function(void * message_memory)
{
  slam_msgs__msg__SlamInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_member_array[3] = {
  {
    "num_maps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(slam_msgs__msg__SlamInfo, num_maps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_keyframes_in_current_map",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(slam_msgs__msg__SlamInfo, num_keyframes_in_current_map),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracking_frequency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(slam_msgs__msg__SlamInfo, tracking_frequency),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_members = {
  "slam_msgs__msg",  // message namespace
  "SlamInfo",  // message name
  3,  // number of fields
  sizeof(slam_msgs__msg__SlamInfo),
  slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_member_array,  // message members
  slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_type_support_handle = {
  0,
  &slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_slam_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, slam_msgs, msg, SlamInfo)() {
  if (!slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_type_support_handle.typesupport_identifier) {
    slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &slam_msgs__msg__SlamInfo__rosidl_typesupport_introspection_c__SlamInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
