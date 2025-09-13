// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from communication_msgs:msg/TaskAllocationArray.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__STRUCT_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tasks'
#include "communication_msgs/msg/detail/task_allocation__struct.h"

/// Struct defined in msg/TaskAllocationArray in the package communication_msgs.
typedef struct communication_msgs__msg__TaskAllocationArray
{
  communication_msgs__msg__TaskAllocation__Sequence tasks;
} communication_msgs__msg__TaskAllocationArray;

// Struct for a sequence of communication_msgs__msg__TaskAllocationArray.
typedef struct communication_msgs__msg__TaskAllocationArray__Sequence
{
  communication_msgs__msg__TaskAllocationArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} communication_msgs__msg__TaskAllocationArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__STRUCT_H_
