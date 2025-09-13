// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_Task_available
{
public:
  explicit Init_Task_available(::communication_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  ::communication_msgs::msg::Task available(::communication_msgs::msg::Task::_available_type arg)
  {
    msg_.available = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::Task msg_;
};

class Init_Task_dropoff_y
{
public:
  explicit Init_Task_dropoff_y(::communication_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_available dropoff_y(::communication_msgs::msg::Task::_dropoff_y_type arg)
  {
    msg_.dropoff_y = std::move(arg);
    return Init_Task_available(msg_);
  }

private:
  ::communication_msgs::msg::Task msg_;
};

class Init_Task_dropoff_x
{
public:
  explicit Init_Task_dropoff_x(::communication_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_dropoff_y dropoff_x(::communication_msgs::msg::Task::_dropoff_x_type arg)
  {
    msg_.dropoff_x = std::move(arg);
    return Init_Task_dropoff_y(msg_);
  }

private:
  ::communication_msgs::msg::Task msg_;
};

class Init_Task_pickup_y
{
public:
  explicit Init_Task_pickup_y(::communication_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_dropoff_x pickup_y(::communication_msgs::msg::Task::_pickup_y_type arg)
  {
    msg_.pickup_y = std::move(arg);
    return Init_Task_dropoff_x(msg_);
  }

private:
  ::communication_msgs::msg::Task msg_;
};

class Init_Task_pickup_x
{
public:
  explicit Init_Task_pickup_x(::communication_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_pickup_y pickup_x(::communication_msgs::msg::Task::_pickup_x_type arg)
  {
    msg_.pickup_x = std::move(arg);
    return Init_Task_pickup_y(msg_);
  }

private:
  ::communication_msgs::msg::Task msg_;
};

class Init_Task_task_id
{
public:
  Init_Task_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_pickup_x task_id(::communication_msgs::msg::Task::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_Task_pickup_x(msg_);
  }

private:
  ::communication_msgs::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::Task>()
{
  return communication_msgs::msg::builder::Init_Task_task_id();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK__BUILDER_HPP_
