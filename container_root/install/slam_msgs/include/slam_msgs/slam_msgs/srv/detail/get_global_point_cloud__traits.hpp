// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from slam_msgs:srv/GetGlobalPointCloud.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__TRAITS_HPP_
#define SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "slam_msgs/srv/detail/get_global_point_cloud__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace slam_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetGlobalPointCloud_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: global_voxel_resolution
  {
    out << "global_voxel_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.global_voxel_resolution, out);
    out << ", ";
  }

  // member: local_voxel_resolution
  {
    out << "local_voxel_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.local_voxel_resolution, out);
    out << ", ";
  }

  // member: z_thresh_max
  {
    out << "z_thresh_max: ";
    rosidl_generator_traits::value_to_yaml(msg.z_thresh_max, out);
    out << ", ";
  }

  // member: get_grayscale
  {
    out << "get_grayscale: ";
    rosidl_generator_traits::value_to_yaml(msg.get_grayscale, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetGlobalPointCloud_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: global_voxel_resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_voxel_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.global_voxel_resolution, out);
    out << "\n";
  }

  // member: local_voxel_resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_voxel_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.local_voxel_resolution, out);
    out << "\n";
  }

  // member: z_thresh_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_thresh_max: ";
    rosidl_generator_traits::value_to_yaml(msg.z_thresh_max, out);
    out << "\n";
  }

  // member: get_grayscale
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_grayscale: ";
    rosidl_generator_traits::value_to_yaml(msg.get_grayscale, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetGlobalPointCloud_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace slam_msgs

namespace rosidl_generator_traits
{

[[deprecated("use slam_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const slam_msgs::srv::GetGlobalPointCloud_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  slam_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use slam_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const slam_msgs::srv::GetGlobalPointCloud_Request & msg)
{
  return slam_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<slam_msgs::srv::GetGlobalPointCloud_Request>()
{
  return "slam_msgs::srv::GetGlobalPointCloud_Request";
}

template<>
inline const char * name<slam_msgs::srv::GetGlobalPointCloud_Request>()
{
  return "slam_msgs/srv/GetGlobalPointCloud_Request";
}

template<>
struct has_fixed_size<slam_msgs::srv::GetGlobalPointCloud_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<slam_msgs::srv::GetGlobalPointCloud_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<slam_msgs::srv::GetGlobalPointCloud_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace slam_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetGlobalPointCloud_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetGlobalPointCloud_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetGlobalPointCloud_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace slam_msgs

namespace rosidl_generator_traits
{

[[deprecated("use slam_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const slam_msgs::srv::GetGlobalPointCloud_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  slam_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use slam_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const slam_msgs::srv::GetGlobalPointCloud_Response & msg)
{
  return slam_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<slam_msgs::srv::GetGlobalPointCloud_Response>()
{
  return "slam_msgs::srv::GetGlobalPointCloud_Response";
}

template<>
inline const char * name<slam_msgs::srv::GetGlobalPointCloud_Response>()
{
  return "slam_msgs/srv/GetGlobalPointCloud_Response";
}

template<>
struct has_fixed_size<slam_msgs::srv::GetGlobalPointCloud_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<slam_msgs::srv::GetGlobalPointCloud_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<slam_msgs::srv::GetGlobalPointCloud_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<slam_msgs::srv::GetGlobalPointCloud>()
{
  return "slam_msgs::srv::GetGlobalPointCloud";
}

template<>
inline const char * name<slam_msgs::srv::GetGlobalPointCloud>()
{
  return "slam_msgs/srv/GetGlobalPointCloud";
}

template<>
struct has_fixed_size<slam_msgs::srv::GetGlobalPointCloud>
  : std::integral_constant<
    bool,
    has_fixed_size<slam_msgs::srv::GetGlobalPointCloud_Request>::value &&
    has_fixed_size<slam_msgs::srv::GetGlobalPointCloud_Response>::value
  >
{
};

template<>
struct has_bounded_size<slam_msgs::srv::GetGlobalPointCloud>
  : std::integral_constant<
    bool,
    has_bounded_size<slam_msgs::srv::GetGlobalPointCloud_Request>::value &&
    has_bounded_size<slam_msgs::srv::GetGlobalPointCloud_Response>::value
  >
{
};

template<>
struct is_service<slam_msgs::srv::GetGlobalPointCloud>
  : std::true_type
{
};

template<>
struct is_service_request<slam_msgs::srv::GetGlobalPointCloud_Request>
  : std::true_type
{
};

template<>
struct is_service_response<slam_msgs::srv::GetGlobalPointCloud_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__TRAITS_HPP_
