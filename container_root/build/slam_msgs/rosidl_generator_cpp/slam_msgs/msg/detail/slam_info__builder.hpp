// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from slam_msgs:msg/SlamInfo.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__MSG__DETAIL__SLAM_INFO__BUILDER_HPP_
#define SLAM_MSGS__MSG__DETAIL__SLAM_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "slam_msgs/msg/detail/slam_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace slam_msgs
{

namespace msg
{

namespace builder
{

class Init_SlamInfo_tracking_frequency
{
public:
  explicit Init_SlamInfo_tracking_frequency(::slam_msgs::msg::SlamInfo & msg)
  : msg_(msg)
  {}
  ::slam_msgs::msg::SlamInfo tracking_frequency(::slam_msgs::msg::SlamInfo::_tracking_frequency_type arg)
  {
    msg_.tracking_frequency = std::move(arg);
    return std::move(msg_);
  }

private:
  ::slam_msgs::msg::SlamInfo msg_;
};

class Init_SlamInfo_num_keyframes_in_current_map
{
public:
  explicit Init_SlamInfo_num_keyframes_in_current_map(::slam_msgs::msg::SlamInfo & msg)
  : msg_(msg)
  {}
  Init_SlamInfo_tracking_frequency num_keyframes_in_current_map(::slam_msgs::msg::SlamInfo::_num_keyframes_in_current_map_type arg)
  {
    msg_.num_keyframes_in_current_map = std::move(arg);
    return Init_SlamInfo_tracking_frequency(msg_);
  }

private:
  ::slam_msgs::msg::SlamInfo msg_;
};

class Init_SlamInfo_num_maps
{
public:
  Init_SlamInfo_num_maps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SlamInfo_num_keyframes_in_current_map num_maps(::slam_msgs::msg::SlamInfo::_num_maps_type arg)
  {
    msg_.num_maps = std::move(arg);
    return Init_SlamInfo_num_keyframes_in_current_map(msg_);
  }

private:
  ::slam_msgs::msg::SlamInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::slam_msgs::msg::SlamInfo>()
{
  return slam_msgs::msg::builder::Init_SlamInfo_num_maps();
}

}  // namespace slam_msgs

#endif  // SLAM_MSGS__MSG__DETAIL__SLAM_INFO__BUILDER_HPP_
