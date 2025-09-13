// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/TaskAllocationArray.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/task_allocation_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_TaskAllocationArray_tasks
{
public:
  Init_TaskAllocationArray_tasks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::communication_msgs::msg::TaskAllocationArray tasks(::communication_msgs::msg::TaskAllocationArray::_tasks_type arg)
  {
    msg_.tasks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::TaskAllocationArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::TaskAllocationArray>()
{
  return communication_msgs::msg::builder::Init_TaskAllocationArray_tasks();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__BUILDER_HPP_
