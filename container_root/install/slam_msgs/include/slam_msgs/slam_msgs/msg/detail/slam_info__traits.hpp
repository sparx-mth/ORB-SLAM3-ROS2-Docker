// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from slam_msgs:msg/SlamInfo.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__MSG__DETAIL__SLAM_INFO__TRAITS_HPP_
#define SLAM_MSGS__MSG__DETAIL__SLAM_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "slam_msgs/msg/detail/slam_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace slam_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SlamInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: num_maps
  {
    out << "num_maps: ";
    rosidl_generator_traits::value_to_yaml(msg.num_maps, out);
    out << ", ";
  }

  // member: num_keyframes_in_current_map
  {
    out << "num_keyframes_in_current_map: ";
    rosidl_generator_traits::value_to_yaml(msg.num_keyframes_in_current_map, out);
    out << ", ";
  }

  // member: tracking_frequency
  {
    out << "tracking_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_frequency, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SlamInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num_maps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_maps: ";
    rosidl_generator_traits::value_to_yaml(msg.num_maps, out);
    out << "\n";
  }

  // member: num_keyframes_in_current_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_keyframes_in_current_map: ";
    rosidl_generator_traits::value_to_yaml(msg.num_keyframes_in_current_map, out);
    out << "\n";
  }

  // member: tracking_frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_frequency, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SlamInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace slam_msgs

namespace rosidl_generator_traits
{

[[deprecated("use slam_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const slam_msgs::msg::SlamInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  slam_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use slam_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const slam_msgs::msg::SlamInfo & msg)
{
  return slam_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<slam_msgs::msg::SlamInfo>()
{
  return "slam_msgs::msg::SlamInfo";
}

template<>
inline const char * name<slam_msgs::msg::SlamInfo>()
{
  return "slam_msgs/msg/SlamInfo";
}

template<>
struct has_fixed_size<slam_msgs::msg::SlamInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<slam_msgs::msg::SlamInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<slam_msgs::msg::SlamInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SLAM_MSGS__MSG__DETAIL__SLAM_INFO__TRAITS_HPP_
