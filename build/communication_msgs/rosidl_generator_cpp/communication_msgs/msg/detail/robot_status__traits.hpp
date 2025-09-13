// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from communication_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "communication_msgs/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace communication_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: available
  {
    out << "available: ";
    rosidl_generator_traits::value_to_yaml(msg.available, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "available: ";
    rosidl_generator_traits::value_to_yaml(msg.available, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace communication_msgs

namespace rosidl_generator_traits
{

[[deprecated("use communication_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const communication_msgs::msg::RobotStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  communication_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use communication_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const communication_msgs::msg::RobotStatus & msg)
{
  return communication_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<communication_msgs::msg::RobotStatus>()
{
  return "communication_msgs::msg::RobotStatus";
}

template<>
inline const char * name<communication_msgs::msg::RobotStatus>()
{
  return "communication_msgs/msg/RobotStatus";
}

template<>
struct has_fixed_size<communication_msgs::msg::RobotStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<communication_msgs::msg::RobotStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<communication_msgs::msg::RobotStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
