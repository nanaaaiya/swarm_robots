// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from communication_msgs:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK__STRUCT_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__communication_msgs__msg__Task __attribute__((deprecated))
#else
# define DEPRECATED__communication_msgs__msg__Task __declspec(deprecated)
#endif

namespace communication_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Task_
{
  using Type = Task_<ContainerAllocator>;

  explicit Task_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = 0l;
      this->pickup_x = 0.0f;
      this->pickup_y = 0.0f;
      this->dropoff_x = 0.0f;
      this->dropoff_y = 0.0f;
      this->available = false;
    }
  }

  explicit Task_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = 0l;
      this->pickup_x = 0.0f;
      this->pickup_y = 0.0f;
      this->dropoff_x = 0.0f;
      this->dropoff_y = 0.0f;
      this->available = false;
    }
  }

  // field types and members
  using _task_id_type =
    int32_t;
  _task_id_type task_id;
  using _pickup_x_type =
    float;
  _pickup_x_type pickup_x;
  using _pickup_y_type =
    float;
  _pickup_y_type pickup_y;
  using _dropoff_x_type =
    float;
  _dropoff_x_type dropoff_x;
  using _dropoff_y_type =
    float;
  _dropoff_y_type dropoff_y;
  using _available_type =
    bool;
  _available_type available;

  // setters for named parameter idiom
  Type & set__task_id(
    const int32_t & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__pickup_x(
    const float & _arg)
  {
    this->pickup_x = _arg;
    return *this;
  }
  Type & set__pickup_y(
    const float & _arg)
  {
    this->pickup_y = _arg;
    return *this;
  }
  Type & set__dropoff_x(
    const float & _arg)
  {
    this->dropoff_x = _arg;
    return *this;
  }
  Type & set__dropoff_y(
    const float & _arg)
  {
    this->dropoff_y = _arg;
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
    communication_msgs::msg::Task_<ContainerAllocator> *;
  using ConstRawPtr =
    const communication_msgs::msg::Task_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<communication_msgs::msg::Task_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<communication_msgs::msg::Task_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::Task_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::Task_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::Task_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::Task_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<communication_msgs::msg::Task_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<communication_msgs::msg::Task_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__communication_msgs__msg__Task
    std::shared_ptr<communication_msgs::msg::Task_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__communication_msgs__msg__Task
    std::shared_ptr<communication_msgs::msg::Task_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Task_ & other) const
  {
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->pickup_x != other.pickup_x) {
      return false;
    }
    if (this->pickup_y != other.pickup_y) {
      return false;
    }
    if (this->dropoff_x != other.dropoff_x) {
      return false;
    }
    if (this->dropoff_y != other.dropoff_y) {
      return false;
    }
    if (this->available != other.available) {
      return false;
    }
    return true;
  }
  bool operator!=(const Task_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Task_

// alias to use template instance with default allocator
using Task =
  communication_msgs::msg::Task_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK__STRUCT_HPP_
