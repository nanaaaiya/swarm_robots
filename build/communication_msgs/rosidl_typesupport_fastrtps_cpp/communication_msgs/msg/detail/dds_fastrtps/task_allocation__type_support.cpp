// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from communication_msgs:msg/TaskAllocation.idl
// generated code does not contain a copyright notice
#include "communication_msgs/msg/detail/task_allocation__rosidl_typesupport_fastrtps_cpp.hpp"
#include "communication_msgs/msg/detail/task_allocation__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace communication_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_communication_msgs
cdr_serialize(
  const communication_msgs::msg::TaskAllocation & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: robot_id
  cdr << ros_message.robot_id;
  // Member: task_ids
  {
    cdr << ros_message.task_ids;
  }
  // Member: total_distance
  cdr << ros_message.total_distance;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_communication_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  communication_msgs::msg::TaskAllocation & ros_message)
{
  // Member: robot_id
  cdr >> ros_message.robot_id;

  // Member: task_ids
  {
    cdr >> ros_message.task_ids;
  }

  // Member: total_distance
  cdr >> ros_message.total_distance;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_communication_msgs
get_serialized_size(
  const communication_msgs::msg::TaskAllocation & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: robot_id
  {
    size_t item_size = sizeof(ros_message.robot_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: task_ids
  {
    size_t array_size = ros_message.task_ids.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.task_ids[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_distance
  {
    size_t item_size = sizeof(ros_message.total_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_communication_msgs
max_serialized_size_TaskAllocation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: robot_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: task_ids
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_distance
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = communication_msgs::msg::TaskAllocation;
    is_plain =
      (
      offsetof(DataType, total_distance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _TaskAllocation__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const communication_msgs::msg::TaskAllocation *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TaskAllocation__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<communication_msgs::msg::TaskAllocation *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TaskAllocation__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const communication_msgs::msg::TaskAllocation *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TaskAllocation__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TaskAllocation(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TaskAllocation__callbacks = {
  "communication_msgs::msg",
  "TaskAllocation",
  _TaskAllocation__cdr_serialize,
  _TaskAllocation__cdr_deserialize,
  _TaskAllocation__get_serialized_size,
  _TaskAllocation__max_serialized_size
};

static rosidl_message_type_support_t _TaskAllocation__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TaskAllocation__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace communication_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_communication_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<communication_msgs::msg::TaskAllocation>()
{
  return &communication_msgs::msg::typesupport_fastrtps_cpp::_TaskAllocation__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, communication_msgs, msg, TaskAllocation)() {
  return &communication_msgs::msg::typesupport_fastrtps_cpp::_TaskAllocation__handle;
}

#ifdef __cplusplus
}
#endif
