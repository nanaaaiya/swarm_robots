// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from communication_msgs:msg/TaskAllocation.idl
// generated code does not contain a copyright notice
#include "communication_msgs/msg/detail/task_allocation__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "communication_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "communication_msgs/msg/detail/task_allocation__struct.h"
#include "communication_msgs/msg/detail/task_allocation__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // task_ids
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // task_ids

// forward declare type support functions


using _TaskAllocation__ros_msg_type = communication_msgs__msg__TaskAllocation;

static bool _TaskAllocation__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TaskAllocation__ros_msg_type * ros_message = static_cast<const _TaskAllocation__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    cdr << ros_message->robot_id;
  }

  // Field name: task_ids
  {
    size_t size = ros_message->task_ids.size;
    auto array_ptr = ros_message->task_ids.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: total_distance
  {
    cdr << ros_message->total_distance;
  }

  return true;
}

static bool _TaskAllocation__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TaskAllocation__ros_msg_type * ros_message = static_cast<_TaskAllocation__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    cdr >> ros_message->robot_id;
  }

  // Field name: task_ids
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->task_ids.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->task_ids);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->task_ids, size)) {
      fprintf(stderr, "failed to create array for field 'task_ids'");
      return false;
    }
    auto array_ptr = ros_message->task_ids.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: total_distance
  {
    cdr >> ros_message->total_distance;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_communication_msgs
size_t get_serialized_size_communication_msgs__msg__TaskAllocation(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TaskAllocation__ros_msg_type * ros_message = static_cast<const _TaskAllocation__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robot_id
  {
    size_t item_size = sizeof(ros_message->robot_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name task_ids
  {
    size_t array_size = ros_message->task_ids.size;
    auto array_ptr = ros_message->task_ids.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name total_distance
  {
    size_t item_size = sizeof(ros_message->total_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TaskAllocation__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_communication_msgs__msg__TaskAllocation(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_communication_msgs
size_t max_serialized_size_communication_msgs__msg__TaskAllocation(
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

  // member: robot_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: task_ids
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
  // member: total_distance
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
    using DataType = communication_msgs__msg__TaskAllocation;
    is_plain =
      (
      offsetof(DataType, total_distance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TaskAllocation__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_communication_msgs__msg__TaskAllocation(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TaskAllocation = {
  "communication_msgs::msg",
  "TaskAllocation",
  _TaskAllocation__cdr_serialize,
  _TaskAllocation__cdr_deserialize,
  _TaskAllocation__get_serialized_size,
  _TaskAllocation__max_serialized_size
};

static rosidl_message_type_support_t _TaskAllocation__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TaskAllocation,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, communication_msgs, msg, TaskAllocation)() {
  return &_TaskAllocation__type_support;
}

#if defined(__cplusplus)
}
#endif
