// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from communication_msgs:msg/TaskAllocationArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "communication_msgs/msg/detail/task_allocation_array__struct.hpp"
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

void TaskAllocationArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) communication_msgs::msg::TaskAllocationArray(_init);
}

void TaskAllocationArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<communication_msgs::msg::TaskAllocationArray *>(message_memory);
  typed_message->~TaskAllocationArray();
}

size_t size_function__TaskAllocationArray__tasks(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<communication_msgs::msg::TaskAllocation> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TaskAllocationArray__tasks(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<communication_msgs::msg::TaskAllocation> *>(untyped_member);
  return &member[index];
}

void * get_function__TaskAllocationArray__tasks(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<communication_msgs::msg::TaskAllocation> *>(untyped_member);
  return &member[index];
}

void fetch_function__TaskAllocationArray__tasks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const communication_msgs::msg::TaskAllocation *>(
    get_const_function__TaskAllocationArray__tasks(untyped_member, index));
  auto & value = *reinterpret_cast<communication_msgs::msg::TaskAllocation *>(untyped_value);
  value = item;
}

void assign_function__TaskAllocationArray__tasks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<communication_msgs::msg::TaskAllocation *>(
    get_function__TaskAllocationArray__tasks(untyped_member, index));
  const auto & value = *reinterpret_cast<const communication_msgs::msg::TaskAllocation *>(untyped_value);
  item = value;
}

void resize_function__TaskAllocationArray__tasks(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<communication_msgs::msg::TaskAllocation> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TaskAllocationArray_message_member_array[1] = {
  {
    "tasks",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<communication_msgs::msg::TaskAllocation>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(communication_msgs::msg::TaskAllocationArray, tasks),  // bytes offset in struct
    nullptr,  // default value
    size_function__TaskAllocationArray__tasks,  // size() function pointer
    get_const_function__TaskAllocationArray__tasks,  // get_const(index) function pointer
    get_function__TaskAllocationArray__tasks,  // get(index) function pointer
    fetch_function__TaskAllocationArray__tasks,  // fetch(index, &value) function pointer
    assign_function__TaskAllocationArray__tasks,  // assign(index, value) function pointer
    resize_function__TaskAllocationArray__tasks  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TaskAllocationArray_message_members = {
  "communication_msgs::msg",  // message namespace
  "TaskAllocationArray",  // message name
  1,  // number of fields
  sizeof(communication_msgs::msg::TaskAllocationArray),
  TaskAllocationArray_message_member_array,  // message members
  TaskAllocationArray_init_function,  // function to initialize message memory (memory has to be allocated)
  TaskAllocationArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TaskAllocationArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TaskAllocationArray_message_members,
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
get_message_type_support_handle<communication_msgs::msg::TaskAllocationArray>()
{
  return &::communication_msgs::msg::rosidl_typesupport_introspection_cpp::TaskAllocationArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, communication_msgs, msg, TaskAllocationArray)() {
  return &::communication_msgs::msg::rosidl_typesupport_introspection_cpp::TaskAllocationArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
