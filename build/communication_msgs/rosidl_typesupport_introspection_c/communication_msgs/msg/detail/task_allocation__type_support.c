// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from communication_msgs:msg/TaskAllocation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "communication_msgs/msg/detail/task_allocation__rosidl_typesupport_introspection_c.h"
#include "communication_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "communication_msgs/msg/detail/task_allocation__functions.h"
#include "communication_msgs/msg/detail/task_allocation__struct.h"


// Include directives for member types
// Member `task_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  communication_msgs__msg__TaskAllocation__init(message_memory);
}

void communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_fini_function(void * message_memory)
{
  communication_msgs__msg__TaskAllocation__fini(message_memory);
}

size_t communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__size_function__TaskAllocation__task_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__get_const_function__TaskAllocation__task_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__get_function__TaskAllocation__task_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__fetch_function__TaskAllocation__task_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__get_const_function__TaskAllocation__task_ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__assign_function__TaskAllocation__task_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__get_function__TaskAllocation__task_ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__resize_function__TaskAllocation__task_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_member_array[3] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(communication_msgs__msg__TaskAllocation, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(communication_msgs__msg__TaskAllocation, task_ids),  // bytes offset in struct
    NULL,  // default value
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__size_function__TaskAllocation__task_ids,  // size() function pointer
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__get_const_function__TaskAllocation__task_ids,  // get_const(index) function pointer
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__get_function__TaskAllocation__task_ids,  // get(index) function pointer
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__fetch_function__TaskAllocation__task_ids,  // fetch(index, &value) function pointer
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__assign_function__TaskAllocation__task_ids,  // assign(index, value) function pointer
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__resize_function__TaskAllocation__task_ids  // resize(index) function pointer
  },
  {
    "total_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(communication_msgs__msg__TaskAllocation, total_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_members = {
  "communication_msgs__msg",  // message namespace
  "TaskAllocation",  // message name
  3,  // number of fields
  sizeof(communication_msgs__msg__TaskAllocation),
  communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_member_array,  // message members
  communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_init_function,  // function to initialize message memory (memory has to be allocated)
  communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_type_support_handle = {
  0,
  &communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_communication_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, communication_msgs, msg, TaskAllocation)() {
  if (!communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_type_support_handle.typesupport_identifier) {
    communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &communication_msgs__msg__TaskAllocation__rosidl_typesupport_introspection_c__TaskAllocation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
