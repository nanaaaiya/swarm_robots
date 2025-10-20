// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from communication_msgs:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOT__TRAITS_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "communication_msgs/msg/detail/robot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace communication_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robot & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
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
  const Robot & msg,
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

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
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

inline std::string to_yaml(const Robot & msg, bool use_flow_style = false)
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
  const communication_msgs::msg::Robot & msg,
  std::ostream & out, size_t indentation = 0)
{
  communication_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use communication_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const communication_msgs::msg::Robot & msg)
{
  return communication_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<communication_msgs::msg::Robot>()
{
  return "communication_msgs::msg::Robot";
}

template<>
inline const char * name<communication_msgs::msg::Robot>()
{
  return "communication_msgs/msg/Robot";
}

template<>
struct has_fixed_size<communication_msgs::msg::Robot>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<communication_msgs::msg::Robot>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<communication_msgs::msg::Robot>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOT__TRAITS_HPP_
