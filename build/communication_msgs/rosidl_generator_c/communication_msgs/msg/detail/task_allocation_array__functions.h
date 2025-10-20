// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from communication_msgs:msg/TaskAllocationArray.idl
// generated code does not contain a copyright notice

#ifndef COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__FUNCTIONS_H_
#define COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "communication_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "communication_msgs/msg/detail/task_allocation_array__struct.h"

/// Initialize msg/TaskAllocationArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * communication_msgs__msg__TaskAllocationArray
 * )) before or use
 * communication_msgs__msg__TaskAllocationArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__TaskAllocationArray__init(communication_msgs__msg__TaskAllocationArray * msg);

/// Finalize msg/TaskAllocationArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__TaskAllocationArray__fini(communication_msgs__msg__TaskAllocationArray * msg);

/// Create msg/TaskAllocationArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * communication_msgs__msg__TaskAllocationArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
communication_msgs__msg__TaskAllocationArray *
communication_msgs__msg__TaskAllocationArray__create();

/// Destroy msg/TaskAllocationArray message.
/**
 * It calls
 * communication_msgs__msg__TaskAllocationArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__TaskAllocationArray__destroy(communication_msgs__msg__TaskAllocationArray * msg);

/// Check for msg/TaskAllocationArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__TaskAllocationArray__are_equal(const communication_msgs__msg__TaskAllocationArray * lhs, const communication_msgs__msg__TaskAllocationArray * rhs);

/// Copy a msg/TaskAllocationArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__TaskAllocationArray__copy(
  const communication_msgs__msg__TaskAllocationArray * input,
  communication_msgs__msg__TaskAllocationArray * output);

/// Initialize array of msg/TaskAllocationArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * communication_msgs__msg__TaskAllocationArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__TaskAllocationArray__Sequence__init(communication_msgs__msg__TaskAllocationArray__Sequence * array, size_t size);

/// Finalize array of msg/TaskAllocationArray messages.
/**
 * It calls
 * communication_msgs__msg__TaskAllocationArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__TaskAllocationArray__Sequence__fini(communication_msgs__msg__TaskAllocationArray__Sequence * array);

/// Create array of msg/TaskAllocationArray messages.
/**
 * It allocates the memory for the array and calls
 * communication_msgs__msg__TaskAllocationArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
communication_msgs__msg__TaskAllocationArray__Sequence *
communication_msgs__msg__TaskAllocationArray__Sequence__create(size_t size);

/// Destroy array of msg/TaskAllocationArray messages.
/**
 * It calls
 * communication_msgs__msg__TaskAllocationArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
void
communication_msgs__msg__TaskAllocationArray__Sequence__destroy(communication_msgs__msg__TaskAllocationArray__Sequence * array);

/// Check for msg/TaskAllocationArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__TaskAllocationArray__Sequence__are_equal(const communication_msgs__msg__TaskAllocationArray__Sequence * lhs, const communication_msgs__msg__TaskAllocationArray__Sequence * rhs);

/// Copy an array of msg/TaskAllocationArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_communication_msgs
bool
communication_msgs__msg__TaskAllocationArray__Sequence__copy(
  const communication_msgs__msg__TaskAllocationArray__Sequence * input,
  communication_msgs__msg__TaskAllocationArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // COMMUNICATION_MSGS__MSG__DETAIL__TASK_ALLOCATION_ARRAY__FUNCTIONS_H_
