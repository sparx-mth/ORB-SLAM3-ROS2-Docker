// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from slam_msgs:srv/GetAllLandmarksInMap.idl
// generated code does not contain a copyright notice

#ifndef SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__STRUCT_HPP_
#define SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Request __declspec(deprecated)
#endif

namespace slam_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAllLandmarksInMap_Request_
{
  using Type = GetAllLandmarksInMap_Request_<ContainerAllocator>;

  explicit GetAllLandmarksInMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->request = true;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->request = false;
    }
  }

  explicit GetAllLandmarksInMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->request = true;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->request = false;
    }
  }

  // field types and members
  using _request_type =
    bool;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const bool & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Request
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Request
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAllLandmarksInMap_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAllLandmarksInMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAllLandmarksInMap_Request_

// alias to use template instance with default allocator
using GetAllLandmarksInMap_Request =
  slam_msgs::srv::GetAllLandmarksInMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace slam_msgs


// Include directives for member types
// Member 'landmarks'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Response __declspec(deprecated)
#endif

namespace slam_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAllLandmarksInMap_Response_
{
  using Type = GetAllLandmarksInMap_Response_<ContainerAllocator>;

  explicit GetAllLandmarksInMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : landmarks(_init)
  {
    (void)_init;
  }

  explicit GetAllLandmarksInMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : landmarks(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _landmarks_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _landmarks_type landmarks;

  // setters for named parameter idiom
  Type & set__landmarks(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->landmarks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Response
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__slam_msgs__srv__GetAllLandmarksInMap_Response
    std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAllLandmarksInMap_Response_ & other) const
  {
    if (this->landmarks != other.landmarks) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAllLandmarksInMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAllLandmarksInMap_Response_

// alias to use template instance with default allocator
using GetAllLandmarksInMap_Response =
  slam_msgs::srv::GetAllLandmarksInMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace slam_msgs

namespace slam_msgs
{

namespace srv
{

struct GetAllLandmarksInMap
{
  using Request = slam_msgs::srv::GetAllLandmarksInMap_Request;
  using Response = slam_msgs::srv::GetAllLandmarksInMap_Response;
};

}  // namespace srv

}  // namespace slam_msgs

#endif  // SLAM_MSGS__SRV__DETAIL__GET_ALL_LANDMARKS_IN_MAP__STRUCT_HPP_
