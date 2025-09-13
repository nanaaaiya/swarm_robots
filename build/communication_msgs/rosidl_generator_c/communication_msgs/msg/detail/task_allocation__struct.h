// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from communication_msgs:msg/TaskAllocation.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__STRUCT_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task_ids'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/TaskAllocation in the package communication_msgs.
typedef struct communication_msgs__msg__TaskAllocation
{
  int32_t robot_id;
  rosidl_runtime_c__int32__Sequence task_ids;
  float total_distance;
} communication_msgs__msg__TaskAllocation;

// Struct for a sequence of communication_msgs__msg__TaskAllocation.
typedef struct communication_msgs__msg__TaskAllocation__Sequence
{
  communication_msgs__msg__TaskAllocation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} communication_msgs__msg__TaskAllocation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION__STRUCT_H_
