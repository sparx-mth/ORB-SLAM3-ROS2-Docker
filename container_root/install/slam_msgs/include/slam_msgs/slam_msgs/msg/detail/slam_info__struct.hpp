// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from slam_msgs:msg/SlamInfo.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__MSG__DETAIL__SLAM_INFO__STRUCT_HPP_
#define SLAM_MSGS__MSG__DETAIL__SLAM_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__slam_msgs__msg__SlamInfo __attribute__((deprecated))
#else
# define DEPRECATED__slam_msgs__msg__SlamInfo __declspec(deprecated)
#endif

namespace slam_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SlamInfo_
{
  using Type = SlamInfo_<ContainerAllocator>;

  explicit SlamInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_maps = 0l;
      this->num_keyframes_in_current_map = 0l;
      this->tracking_frequency = 0.0f;
    }
  }

  explicit SlamInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_maps = 0l;
      this->num_keyframes_in_current_map = 0l;
      this->tracking_frequency = 0.0f;
    }
  }

  // field types and members
  using _num_maps_type =
    int32_t;
  _num_maps_type num_maps;
  using _num_keyframes_in_current_map_type =
    int32_t;
  _num_keyframes_in_current_map_type num_keyframes_in_current_map;
  using _tracking_frequency_type =
    float;
  _tracking_frequency_type tracking_frequency;

  // setters for named parameter idiom
  Type & set__num_maps(
    const int32_t & _arg)
  {
    this->num_maps = _arg;
    return *this;
  }
  Type & set__num_keyframes_in_current_map(
    const int32_t & _arg)
  {
    this->num_keyframes_in_current_map = _arg;
    return *this;
  }
  Type & set__tracking_frequency(
    const float & _arg)
  {
    this->tracking_frequency = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    slam_msgs::msg::SlamInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const slam_msgs::msg::SlamInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      slam_msgs::msg::SlamInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      slam_msgs::msg::SlamInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__slam_msgs__msg__SlamInfo
    std::shared_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__slam_msgs__msg__SlamInfo
    std::shared_ptr<slam_msgs::msg::SlamInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SlamInfo_ & other) const
  {
    if (this->num_maps != other.num_maps) {
      return false;
    }
    if (this->num_keyframes_in_current_map != other.num_keyframes_in_current_map) {
      return false;
    }
    if (this->tracking_frequency != other.tracking_frequency) {
      return false;
    }
    return true;
  }
  bool operator!=(const SlamInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SlamInfo_

// alias to use template instance with default allocator
using SlamInfo =
  slam_msgs::msg::SlamInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace slam_msgs

#endif  // SLAM_MSGS__MSG__DETAIL__SLAM_INFO__STRUCT_HPP_
