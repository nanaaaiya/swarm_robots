// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from communication_msgs:msg/Robots.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__STRUCT_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_status'
#include "communication_msgs/msg/detail/robot__struct.h"

/// Struct defined in msg/Robots in the package communication_msgs.
typedef struct communication_msgs__msg__Robots
{
  communication_msgs__msg__Robot__Sequence robot_status;
} communication_msgs__msg__Robots;

// Struct for a sequence of communication_msgs__msg__Robots.
typedef struct communication_msgs__msg__Robots__Sequence
{
  communication_msgs__msg__Robots * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} communication_msgs__msg__Robots__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOTS__STRUCT_H_
