// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from slam_msgs:srv/GetGlobalPointCloud.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__STRUCT_HPP_
#define SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Request __attribute__((deprecated))
#else
# define DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Request __declspec(deprecated)
#endif

namespace slam_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGlobalPointCloud_Request_
{
  using Type = GetGlobalPointCloud_Request_<ContainerAllocator>;

  explicit GetGlobalPointCloud_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->global_voxel_resolution = 0.3f;
      this->local_voxel_resolution = 0.3f;
      this->z_thresh_max = 2.0f;
      this->get_grayscale = true;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->global_voxel_resolution = 0.0f;
      this->local_voxel_resolution = 0.0f;
      this->z_thresh_max = 0.0f;
      this->get_grayscale = false;
    }
  }

  explicit GetGlobalPointCloud_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->global_voxel_resolution = 0.3f;
      this->local_voxel_resolution = 0.3f;
      this->z_thresh_max = 2.0f;
      this->get_grayscale = true;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->global_voxel_resolution = 0.0f;
      this->local_voxel_resolution = 0.0f;
      this->z_thresh_max = 0.0f;
      this->get_grayscale = false;
    }
  }

  // field types and members
  using _global_voxel_resolution_type =
    float;
  _global_voxel_resolution_type global_voxel_resolution;
  using _local_voxel_resolution_type =
    float;
  _local_voxel_resolution_type local_voxel_resolution;
  using _z_thresh_max_type =
    float;
  _z_thresh_max_type z_thresh_max;
  using _get_grayscale_type =
    bool;
  _get_grayscale_type get_grayscale;

  // setters for named parameter idiom
  Type & set__global_voxel_resolution(
    const float & _arg)
  {
    this->global_voxel_resolution = _arg;
    return *this;
  }
  Type & set__local_voxel_resolution(
    const float & _arg)
  {
    this->local_voxel_resolution = _arg;
    return *this;
  }
  Type & set__z_thresh_max(
    const float & _arg)
  {
    this->z_thresh_max = _arg;
    return *this;
  }
  Type & set__get_grayscale(
    const bool & _arg)
  {
    this->get_grayscale = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Request
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Request
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGlobalPointCloud_Request_ & other) const
  {
    if (this->global_voxel_resolution != other.global_voxel_resolution) {
      return false;
    }
    if (this->local_voxel_resolution != other.local_voxel_resolution) {
      return false;
    }
    if (this->z_thresh_max != other.z_thresh_max) {
      return false;
    }
    if (this->get_grayscale != other.get_grayscale) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGlobalPointCloud_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGlobalPointCloud_Request_

// alias to use template instance with default allocator
using GetGlobalPointCloud_Request =
  slam_msgs::srv::GetGlobalPointCloud_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace slam_msgs


#ifndef _WIN32
# define DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Response __attribute__((deprecated))
#else
# define DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Response __declspec(deprecated)
#endif

namespace slam_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGlobalPointCloud_Response_
{
  using Type = GetGlobalPointCloud_Response_<ContainerAllocator>;

  explicit GetGlobalPointCloud_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = false;
    }
  }

  explicit GetGlobalPointCloud_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = false;
    }
  }

  // field types and members
  using _response_type =
    bool;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const bool & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Response
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__slam_msgs__srv__GetGlobalPointCloud_Response
    std::shared_ptr<slam_msgs::srv::GetGlobalPointCloud_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGlobalPointCloud_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGlobalPointCloud_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGlobalPointCloud_Response_

// alias to use template instance with default allocator
using GetGlobalPointCloud_Response =
  slam_msgs::srv::GetGlobalPointCloud_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace slam_msgs

namespace slam_msgs
{

namespace srv
{

struct GetGlobalPointCloud
{
  using Request = slam_msgs::srv::GetGlobalPointCloud_Request;
  using Response = slam_msgs::srv::GetGlobalPointCloud_Response;
};

}  // namespace srv

}  // namespace slam_msgs

#endif  // SLAM_MSGS__SRV__DETAIL__GET_GLOBAL_POINT_CLOUD__STRUCT_HPP_
