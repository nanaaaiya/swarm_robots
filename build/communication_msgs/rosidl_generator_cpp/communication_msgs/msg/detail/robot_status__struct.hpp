// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from communication_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__communication_msgs__msg__RobotStatus __attribute__((deprecated))
#else
# define DEPRECATED__communication_msgs__msg__RobotStatus __declspec(deprecated)
#endif

namespace communication_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotStatus_
{
  using Type = RobotStatus_<ContainerAllocator>;

  explicit RobotStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->available = false;
    }
  }

  explicit RobotStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->available = false;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _available_type =
    bool;
  _available_type available;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__available(
    const bool & _arg)
  {
    this->available = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    communication_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const communication_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__communication_msgs__msg__RobotStatus
    std::shared_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__communication_msgs__msg__RobotStatus
    std::shared_ptr<communication_msgs::msg::RobotStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotStatus_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->available != other.available) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotStatus_

// alias to use template instance with default allocator
using RobotStatus =
  communication_msgs::msg::RobotStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
