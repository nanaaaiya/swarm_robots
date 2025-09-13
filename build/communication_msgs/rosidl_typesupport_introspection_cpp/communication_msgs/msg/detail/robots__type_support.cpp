// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from communication_msgs:msg/Robots.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "communication_msgs/msg/detail/robots__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace communication_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Robots_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) communication_msgs::msg::Robots(_init);
}

void Robots_fini_function(void * message_memory)
{
  auto typed_message = static_cast<communication_msgs::msg::Robots *>(message_memory);
  typed_message->~Robots();
}

size_t size_function__Robots__robot_status(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<communication_msgs::msg::Robot> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Robots__robot_status(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<communication_msgs::msg::Robot> *>(untyped_member);
  return &member[index];
}

void * get_function__Robots__robot_status(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<communication_msgs::msg::Robot> *>(untyped_member);
  return &member[index];
}

void fetch_function__Robots__robot_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const communication_msgs::msg::Robot *>(
    get_const_function__Robots__robot_status(untyped_member, index));
  auto & value = *reinterpret_cast<communication_msgs::msg::Robot *>(untyped_value);
  value = item;
}

void assign_function__Robots__robot_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<communication_msgs::msg::Robot *>(
    get_function__Robots__robot_status(untyped_member, index));
  const auto & value = *reinterpret_cast<const communication_msgs::msg::Robot *>(untyped_value);
  item = value;
}

void resize_function__Robots__robot_status(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<communication_msgs::msg::Robot> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Robots_message_member_array[1] = {
  {
    "robot_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<communication_msgs::msg::Robot>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(communication_msgs::msg::Robots, robot_status),  // bytes offset in struct
    nullptr,  // default value
    size_function__Robots__robot_status,  // size() function pointer
    get_const_function__Robots__robot_status,  // get_const(index) function pointer
    get_function__Robots__robot_status,  // get(index) function pointer
    fetch_function__Robots__robot_status,  // fetch(index, &value) function pointer
    assign_function__Robots__robot_status,  // assign(index, value) function pointer
    resize_function__Robots__robot_status  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Robots_message_members = {
  "communication_msgs::msg",  // message namespace
  "Robots",  // message name
  1,  // number of fields
  sizeof(communication_msgs::msg::Robots),
  Robots_message_member_array,  // message members
  Robots_init_function,  // function to initialize message memory (memory has to be allocated)
  Robots_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Robots_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Robots_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace communication_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<communication_msgs::msg::Robots>()
{
  return &::communication_msgs::msg::rosidl_typesupport_introspection_cpp::Robots_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, communication_msgs, msg, Robots)() {
  return &::communication_msgs::msg::rosidl_typesupport_introspection_cpp::Robots_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
