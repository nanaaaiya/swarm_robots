// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from communication_msgs:msg/Robot.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "communication_msgs/msg/detail/robot__struct.h"
#include "communication_msgs/msg/detail/robot__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace communication_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _Robot_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Robot_type_support_ids_t;

static const _Robot_type_support_ids_t _Robot_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Robot_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Robot_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Robot_type_support_symbol_names_t _Robot_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, communication_msgs, msg, Robot)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, communication_msgs, msg, Robot)),
  }
};

typedef struct _Robot_type_support_data_t
{
  void * data[2];
} _Robot_type_support_data_t;

static _Robot_type_support_data_t _Robot_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Robot_message_typesupport_map = {
  2,
  "communication_msgs",
  &_Robot_message_typesupport_ids.typesupport_identifier[0],
  &_Robot_message_typesupport_symbol_names.symbol_name[0],
  &_Robot_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Robot_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Robot_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace communication_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, communication_msgs, msg, Robot)() {
  return &::communication_msgs::msg::rosidl_typesupport_c::Robot_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
