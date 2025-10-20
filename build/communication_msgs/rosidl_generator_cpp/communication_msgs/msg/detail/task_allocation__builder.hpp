// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/TaskAllocation.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/task_allocation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_TaskAllocation_total_distance
{
public:
  explicit Init_TaskAllocation_total_distance(::communication_msgs::msg::TaskAllocation & msg)
  : msg_(msg)
  {}
  ::communication_msgs::msg::TaskAllocation total_distance(::communication_msgs::msg::TaskAllocation::_total_distance_type arg)
  {
    msg_.total_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::TaskAllocation msg_;
};

class Init_TaskAllocation_task_ids
{
public:
  explicit Init_TaskAllocation_task_ids(::communication_msgs::msg::TaskAllocation & msg)
  : msg_(msg)
  {}
  Init_TaskAllocation_total_distance task_ids(::communication_msgs::msg::TaskAllocation::_task_ids_type arg)
  {
    msg_.task_ids = std::move(arg);
    return Init_TaskAllocation_total_distance(msg_);
  }

private:
  ::communication_msgs::msg::TaskAllocation msg_;
};

class Init_TaskAllocation_robot_id
{
public:
  Init_TaskAllocation_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskAllocation_task_ids robot_id(::communication_msgs::msg::TaskAllocation::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_TaskAllocation_task_ids(msg_);
  }

private:
  ::communication_msgs::msg::TaskAllocation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::TaskAllocation>()
{
  return communication_msgs::msg::builder::Init_TaskAllocation_robot_id();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__BUILDER_HPP_
