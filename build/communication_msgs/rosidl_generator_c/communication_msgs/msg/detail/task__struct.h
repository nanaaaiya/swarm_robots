// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from communication_msgs:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK__STRUCT_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Task in the package communication_msgs.
typedef struct communication_msgs__msg__Task
{
  int32_t task_id;
  float pickup_x;
  float pickup_y;
  float dropoff_x;
  float dropoff_y;
  bool available;
} communication_msgs__msg__Task;

// Struct for a sequence of communication_msgs__msg__Task.
typedef struct communication_msgs__msg__Task__Sequence
{
  communication_msgs__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} communication_msgs__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK__STRUCT_H_
