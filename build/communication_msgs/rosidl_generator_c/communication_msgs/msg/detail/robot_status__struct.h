// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from communication_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RobotStatus in the package communication_msgs.
typedef struct communication_msgs__msg__RobotStatus
{
  int32_t robot_id;
  bool available;
} communication_msgs__msg__RobotStatus;

// Struct for a sequence of communication_msgs__msg__RobotStatus.
typedef struct communication_msgs__msg__RobotStatus__Sequence
{
  communication_msgs__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} communication_msgs__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
