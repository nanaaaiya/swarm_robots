// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/RobotOdom.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_ODOM__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_ODOM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/robot_odom__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotOdom_theta
{
public:
  explicit Init_RobotOdom_theta(::communication_msgs::msg::RobotOdom & msg)
  : msg_(msg)
  {}
  ::communication_msgs::msg::RobotOdom theta(::communication_msgs::msg::RobotOdom::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::RobotOdom msg_;
};

class Init_RobotOdom_y
{
public:
  explicit Init_RobotOdom_y(::communication_msgs::msg::RobotOdom & msg)
  : msg_(msg)
  {}
  Init_RobotOdom_theta y(::communication_msgs::msg::RobotOdom::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobotOdom_theta(msg_);
  }

private:
  ::communication_msgs::msg::RobotOdom msg_;
};

class Init_RobotOdom_x
{
public:
  explicit Init_RobotOdom_x(::communication_msgs::msg::RobotOdom & msg)
  : msg_(msg)
  {}
  Init_RobotOdom_y x(::communication_msgs::msg::RobotOdom::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotOdom_y(msg_);
  }

private:
  ::communication_msgs::msg::RobotOdom msg_;
};

class Init_RobotOdom_robot_id
{
public:
  Init_RobotOdom_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotOdom_x robot_id(::communication_msgs::msg::RobotOdom::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotOdom_x(msg_);
  }

private:
  ::communication_msgs::msg::RobotOdom msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::RobotOdom>()
{
  return communication_msgs::msg::builder::Init_RobotOdom_robot_id();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_ODOM__BUILDER_HPP_
