// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from slam_msgs:srv/GetAllLandmarksInMap.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__BUILDER_HPP_
#define SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "slam_msgs/srv/detail/get_all_landmarks_in_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace slam_msgs
{

namespace srv
{

namespace builder
{

class Init_GetAllLandmarksInMap_Request_request
{
public:
  Init_GetAllLandmarksInMap_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::slam_msgs::srv::GetAllLandmarksInMap_Request request(::slam_msgs::srv::GetAllLandmarksInMap_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::slam_msgs::srv::GetAllLandmarksInMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::slam_msgs::srv::GetAllLandmarksInMap_Request>()
{
  return slam_msgs::srv::builder::Init_GetAllLandmarksInMap_Request_request();
}

}  // namespace slam_msgs


namespace slam_msgs
{

namespace srv
{

namespace builder
{

class Init_GetAllLandmarksInMap_Response_landmarks
{
public:
  Init_GetAllLandmarksInMap_Response_landmarks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::slam_msgs::srv::GetAllLandmarksInMap_Response landmarks(::slam_msgs::srv::GetAllLandmarksInMap_Response::_landmarks_type arg)
  {
    msg_.landmarks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::slam_msgs::srv::GetAllLandmarksInMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::slam_msgs::srv::GetAllLandmarksInMap_Response>()
{
  return slam_msgs::srv::builder::Init_GetAllLandmarksInMap_Response_landmarks();
}

}  // namespace slam_msgs

#endif  // SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__BUILDER_HPP_
