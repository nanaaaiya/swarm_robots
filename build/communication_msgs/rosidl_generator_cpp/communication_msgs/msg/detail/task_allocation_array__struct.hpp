// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from communication_msgs:msg/TaskAllocationArray.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__STRUCT_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tasks'
#include "communication_msgs/msg/detail/task_allocation__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__communication_msgs__msg__TaskAllocationArray __attribute__((deprecated))
#else
# define DEPRECATED__communication_msgs__msg__TaskAllocationArray __declspec(deprecated)
#endif

namespace communication_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskAllocationArray_
{
  using Type = TaskAllocationArray_<ContainerAllocator>;

  explicit TaskAllocationArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TaskAllocationArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _tasks_type =
    std::vector<communication_msgs::msg::TaskAllocation_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<communication_msgs::msg::TaskAllocation_<ContainerAllocator>>>;
  _tasks_type tasks;

  // setters for named parameter idiom
  Type & set__tasks(
    const std::vector<communication_msgs::msg::TaskAllocation_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<communication_msgs::msg::TaskAllocation_<ContainerAllocator>>> & _arg)
  {
    this->tasks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    communication_msgs::msg::TaskAllocationArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const communication_msgs::msg::TaskAllocationArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::TaskAllocationArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      communication_msgs::msg::TaskAllocationArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__communication_msgs__msg__TaskAllocationArray
    std::shared_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__communication_msgs__msg__TaskAllocationArray
    std::shared_ptr<communication_msgs::msg::TaskAllocationArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskAllocationArray_ & other) const
  {
    if (this->tasks != other.tasks) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskAllocationArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskAllocationArray_

// alias to use template instance with default allocator
using TaskAllocationArray =
  communication_msgs::msg::TaskAllocationArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__STRUCT_HPP_
