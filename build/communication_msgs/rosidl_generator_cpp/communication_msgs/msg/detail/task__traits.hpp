// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from communication_msgs:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK__TRAITS_HPP_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "communication_msgs/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace communication_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Task & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: pickup_x
  {
    out << "pickup_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pickup_x, out);
    out << ", ";
  }

  // member: pickup_y
  {
    out << "pickup_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pickup_y, out);
    out << ", ";
  }

  // member: dropoff_x
  {
    out << "dropoff_x: ";
    rosidl_generator_traits::value_to_yaml(msg.dropoff_x, out);
    out << ", ";
  }

  // member: dropoff_y
  {
    out << "dropoff_y: ";
    rosidl_generator_traits::value_to_yaml(msg.dropoff_y, out);
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
  const Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: pickup_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pickup_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pickup_x, out);
    out << "\n";
  }

  // member: pickup_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pickup_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pickup_y, out);
    out << "\n";
  }

  // member: dropoff_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dropoff_x: ";
    rosidl_generator_traits::value_to_yaml(msg.dropoff_x, out);
    out << "\n";
  }

  // member: dropoff_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dropoff_y: ";
    rosidl_generator_traits::value_to_yaml(msg.dropoff_y, out);
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

inline std::string to_yaml(const Task & msg, bool use_flow_style = false)
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
  const communication_msgs::msg::Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  communication_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use communication_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const communication_msgs::msg::Task & msg)
{
  return communication_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<communication_msgs::msg::Task>()
{
  return "communication_msgs::msg::Task";
}

template<>
inline const char * name<communication_msgs::msg::Task>()
{
  return "communication_msgs/msg/Task";
}

template<>
struct has_fixed_size<communication_msgs::msg::Task>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<communication_msgs::msg::Task>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<communication_msgs::msg::Task>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK__TRAITS_HPP_
