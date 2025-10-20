// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from communication_msgs:msg/Robots.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "communication_msgs/msg/detail/robots__rosidl_typesupport_introspection_c.h"
#include "communication_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "communication_msgs/msg/detail/robots__functions.h"
#include "communication_msgs/msg/detail/robots__struct.h"


// Include directives for member types
// Member `robot_status`
#include "communication_msgs/msg/robot.h"
// Member `robot_status`
#include "communication_msgs/msg/detail/robot__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  communication_msgs__msg__Robots__init(message_memory);
}

void communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_fini_function(void * message_memory)
{
  communication_msgs__msg__Robots__fini(message_memory);
}

size_t communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__size_function__Robots__robot_status(
  const void * untyped_member)
{
  const communication_msgs__msg__Robot__Sequence * member =
    (const communication_msgs__msg__Robot__Sequence *)(untyped_member);
  return member->size;
}

const void * communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__get_const_function__Robots__robot_status(
  const void * untyped_member, size_t index)
{
  const communication_msgs__msg__Robot__Sequence * member =
    (const communication_msgs__msg__Robot__Sequence *)(untyped_member);
  return &member->data[index];
}

void * communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__get_function__Robots__robot_status(
  void * untyped_member, size_t index)
{
  communication_msgs__msg__Robot__Sequence * member =
    (communication_msgs__msg__Robot__Sequence *)(untyped_member);
  return &member->data[index];
}

void communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__fetch_function__Robots__robot_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const communication_msgs__msg__Robot * item =
    ((const communication_msgs__msg__Robot *)
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__get_const_function__Robots__robot_status(untyped_member, index));
  communication_msgs__msg__Robot * value =
    (communication_msgs__msg__Robot *)(untyped_value);
  *value = *item;
}

void communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__assign_function__Robots__robot_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  communication_msgs__msg__Robot * item =
    ((communication_msgs__msg__Robot *)
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__get_function__Robots__robot_status(untyped_member, index));
  const communication_msgs__msg__Robot * value =
    (const communication_msgs__msg__Robot *)(untyped_value);
  *item = *value;
}

bool communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__resize_function__Robots__robot_status(
  void * untyped_member, size_t size)
{
  communication_msgs__msg__Robot__Sequence * member =
    (communication_msgs__msg__Robot__Sequence *)(untyped_member);
  communication_msgs__msg__Robot__Sequence__fini(member);
  return communication_msgs__msg__Robot__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_member_array[1] = {
  {
    "robot_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(communication_msgs__msg__Robots, robot_status),  // bytes offset in struct
    NULL,  // default value
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__size_function__Robots__robot_status,  // size() function pointer
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__get_const_function__Robots__robot_status,  // get_const(index) function pointer
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__get_function__Robots__robot_status,  // get(index) function pointer
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__fetch_function__Robots__robot_status,  // fetch(index, &value) function pointer
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__assign_function__Robots__robot_status,  // assign(index, value) function pointer
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__resize_function__Robots__robot_status  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_members = {
  "communication_msgs__msg",  // message namespace
  "Robots",  // message name
  1,  // number of fields
  sizeof(communication_msgs__msg__Robots),
  communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_member_array,  // message members
  communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_init_function,  // function to initialize message memory (memory has to be allocated)
  communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_type_support_handle = {
  0,
  &communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_communication_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, communication_msgs, msg, Robots)() {
  communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, communication_msgs, msg, Robot)();
  if (!communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_type_support_handle.typesupport_identifier) {
    communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &communication_msgs__msg__Robots__rosidl_typesupport_introspection_c__Robots_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
