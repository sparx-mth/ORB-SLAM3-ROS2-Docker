// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from slam_msgs:srv/GetGlobalPointCloud.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__BUILDER_HPP_
#define SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "slam_msgs/srv/detail/get_global_point_cloud__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace slam_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGlobalPointCloud_Request_get_grayscale
{
public:
  explicit Init_GetGlobalPointCloud_Request_get_grayscale(::slam_msgs::srv::GetGlobalPointCloud_Request & msg)
  : msg_(msg)
  {}
  ::slam_msgs::srv::GetGlobalPointCloud_Request get_grayscale(::slam_msgs::srv::GetGlobalPointCloud_Request::_get_grayscale_type arg)
  {
    msg_.get_grayscale = std::move(arg);
    return std::move(msg_);
  }

private:
  ::slam_msgs::srv::GetGlobalPointCloud_Request msg_;
};

class Init_GetGlobalPointCloud_Request_z_thresh_max
{
public:
  explicit Init_GetGlobalPointCloud_Request_z_thresh_max(::slam_msgs::srv::GetGlobalPointCloud_Request & msg)
  : msg_(msg)
  {}
  Init_GetGlobalPointCloud_Request_get_grayscale z_thresh_max(::slam_msgs::srv::GetGlobalPointCloud_Request::_z_thresh_max_type arg)
  {
    msg_.z_thresh_max = std::move(arg);
    return Init_GetGlobalPointCloud_Request_get_grayscale(msg_);
  }

private:
  ::slam_msgs::srv::GetGlobalPointCloud_Request msg_;
};

class Init_GetGlobalPointCloud_Request_local_voxel_resolution
{
public:
  explicit Init_GetGlobalPointCloud_Request_local_voxel_resolution(::slam_msgs::srv::GetGlobalPointCloud_Request & msg)
  : msg_(msg)
  {}
  Init_GetGlobalPointCloud_Request_z_thresh_max local_voxel_resolution(::slam_msgs::srv::GetGlobalPointCloud_Request::_local_voxel_resolution_type arg)
  {
    msg_.local_voxel_resolution = std::move(arg);
    return Init_GetGlobalPointCloud_Request_z_thresh_max(msg_);
  }

private:
  ::slam_msgs::srv::GetGlobalPointCloud_Request msg_;
};

class Init_GetGlobalPointCloud_Request_global_voxel_resolution
{
public:
  Init_GetGlobalPointCloud_Request_global_voxel_resolution()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGlobalPointCloud_Request_local_voxel_resolution global_voxel_resolution(::slam_msgs::srv::GetGlobalPointCloud_Request::_global_voxel_resolution_type arg)
  {
    msg_.global_voxel_resolution = std::move(arg);
    return Init_GetGlobalPointCloud_Request_local_voxel_resolution(msg_);
  }

private:
  ::slam_msgs::srv::GetGlobalPointCloud_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::slam_msgs::srv::GetGlobalPointCloud_Request>()
{
  return slam_msgs::srv::builder::Init_GetGlobalPointCloud_Request_global_voxel_resolution();
}

}  // namespace slam_msgs


namespace slam_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGlobalPointCloud_Response_response
{
public:
  Init_GetGlobalPointCloud_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::slam_msgs::srv::GetGlobalPointCloud_Response response(::slam_msgs::srv::GetGlobalPointCloud_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::slam_msgs::srv::GetGlobalPointCloud_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::slam_msgs::srv::GetGlobalPointCloud_Response>()
{
  return slam_msgs::srv::builder::Init_GetGlobalPointCloud_Response_response();
}

}  // namespace slam_msgs

#endif  // SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__BUILDER_HPP_
