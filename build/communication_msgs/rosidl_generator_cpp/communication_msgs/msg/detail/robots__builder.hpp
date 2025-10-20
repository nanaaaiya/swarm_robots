// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/Robots.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/robots__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_Robots_robot_status
{
public:
  Init_Robots_robot_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::communication_msgs::msg::Robots robot_status(::communication_msgs::msg::Robots::_robot_status_type arg)
  {
    msg_.robot_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::Robots msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::Robots>()
{
  return communication_msgs::msg::builder::Init_Robots_robot_status();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__BUILDER_HPP_
