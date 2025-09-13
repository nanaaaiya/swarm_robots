// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from communication_msgs:msg/TaskAllocation.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__STRUCT_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__communication_msgs__msg__TaskAllocation __attribute__((deprecated))
#else
# define DEPRECATED__communication_msgs__msg__TaskAllocation __declspec(deprecated)
#endif

namespace communication_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskAllocation_
{
  using Type = TaskAllocation_<ContainerAllocator>;

  explicit TaskAllocation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->total_distance = 0.0f;
    }
  }

  explicit TaskAllocation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->total_distance = 0.0f;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _task_ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _task_ids_type task_ids;
  using _total_distance_type =
    float;
  _total_distance_type total_distance;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__task_ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->task_ids = _arg;
    return *this;
  }
  Type & set__total_distance(
    const float & _arg)
  {
    this->total_distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    communication_msgs::msg::TaskAllocation_<ContainerAllocator> *;
  using ConstRawPtr =
    const communication_msgs::msg::TaskAllocation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::TaskAllocation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::TaskAllocation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__communication_msgs__msg__TaskAllocation
    std::shared_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__communication_msgs__msg__TaskAllocation
    std::shared_ptr<communication_msgs::msg::TaskAllocation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskAllocation_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->task_ids != other.task_ids) {
      return false;
    }
    if (this->total_distance != other.total_distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskAllocation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskAllocation_

// alias to use template instance with default allocator
using TaskAllocation =
  communication_msgs::msg::TaskAllocation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__STRUCT_HPP_
