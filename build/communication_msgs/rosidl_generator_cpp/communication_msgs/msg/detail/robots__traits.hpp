// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from communication_msgs:msg/Robots.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__TRAITS_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "communication_msgs/msg/detail/robots__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'robot_status'
#include "communication_msgs/msg/detail/robot__traits.hpp"

namespace communication_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robots & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_status
  {
    if (msg.robot_status.size() == 0) {
      out << "robot_status: []";
    } else {
      out << "robot_status: [";
      size_t pending_items = msg.robot_status.size();
      for (auto item : msg.robot_status) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Robots & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_status.size() == 0) {
      out << "robot_status: []\n";
    } else {
      out << "robot_status:\n";
      for (auto item : msg.robot_status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Robots & msg, bool use_flow_style = false)
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
  const communication_msgs::msg::Robots & msg,
  std::ostream & out, size_t indentation = 0)
{
  communication_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use communication_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const communication_msgs::msg::Robots & msg)
{
  return communication_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<communication_msgs::msg::Robots>()
{
  return "communication_msgs::msg::Robots";
}

template<>
inline const char * name<communication_msgs::msg::Robots>()
{
  return "communication_msgs/msg/Robots";
}

template<>
struct has_fixed_size<communication_msgs::msg::Robots>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<communication_msgs::msg::Robots>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<communication_msgs::msg::Robots>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__TRAITS_HPP_
