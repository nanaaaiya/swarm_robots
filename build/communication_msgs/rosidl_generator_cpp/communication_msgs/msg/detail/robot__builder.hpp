// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from communication_msgs:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOT__BUILDER_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "communication_msgs/msg/detail/robot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace communication_msgs
{

namespace msg
{

namespace builder
{

class Init_Robot_available
{
public:
  explicit Init_Robot_available(::communication_msgs::msg::Robot & msg)
  : msg_(msg)
  {}
  ::communication_msgs::msg::Robot available(::communication_msgs::msg::Robot::_available_type arg)
  {
    msg_.available = std::move(arg);
    return std::move(msg_);
  }

private:
  ::communication_msgs::msg::Robot msg_;
};

class Init_Robot_theta
{
public:
  explicit Init_Robot_theta(::communication_msgs::msg::Robot & msg)
  : msg_(msg)
  {}
  Init_Robot_available theta(::communication_msgs::msg::Robot::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Robot_available(msg_);
  }

private:
  ::communication_msgs::msg::Robot msg_;
};

class Init_Robot_y
{
public:
  explicit Init_Robot_y(::communication_msgs::msg::Robot & msg)
  : msg_(msg)
  {}
  Init_Robot_theta y(::communication_msgs::msg::Robot::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Robot_theta(msg_);
  }

private:
  ::communication_msgs::msg::Robot msg_;
};

class Init_Robot_x
{
public:
  explicit Init_Robot_x(::communication_msgs::msg::Robot & msg)
  : msg_(msg)
  {}
  Init_Robot_y x(::communication_msgs::msg::Robot::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Robot_y(msg_);
  }

private:
  ::communication_msgs::msg::Robot msg_;
};

class Init_Robot_robot_id
{
public:
  Init_Robot_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Robot_x robot_id(::communication_msgs::msg::Robot::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Robot_x(msg_);
  }

private:
  ::communication_msgs::msg::Robot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::communication_msgs::msg::Robot>()
{
  return communication_msgs::msg::builder::Init_Robot_robot_id();
}

}  // namespace communication_msgs

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOT__BUILDER_HPP_
