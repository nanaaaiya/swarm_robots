// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from communication_msgs:msg/TaskAllocationArray.idl
// generated code does not contain a copyright notice
#include "communication_msgs/msg/detail/task_allocation_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tasks`
#include "communication_msgs/msg/detail/task_allocation__functions.h"

bool
communication_msgs__msg__TaskAllocationArray__init(communication_msgs__msg__TaskAllocationArray * msg)
{
  if (!msg) {
    return false;
  }
  // tasks
  if (!communication_msgs__msg__TaskAllocation__Sequence__init(&msg->tasks, 0)) {
    communication_msgs__msg__TaskAllocationArray__fini(msg);
    return false;
  }
  return true;
}

void
communication_msgs__msg__TaskAllocationArray__fini(communication_msgs__msg__TaskAllocationArray * msg)
{
  if (!msg) {
    return;
  }
  // tasks
  communication_msgs__msg__TaskAllocation__Sequence__fini(&msg->tasks);
}

bool
communication_msgs__msg__TaskAllocationArray__are_equal(const communication_msgs__msg__TaskAllocationArray * lhs, const communication_msgs__msg__TaskAllocationArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tasks
  if (!communication_msgs__msg__TaskAllocation__Sequence__are_equal(
      &(lhs->tasks), &(rhs->tasks)))
  {
    return false;
  }
  return true;
}

bool
communication_msgs__msg__TaskAllocationArray__copy(
  const communication_msgs__msg__TaskAllocationArray * input,
  communication_msgs__msg__TaskAllocationArray * output)
{
  if (!input || !output) {
    return false;
  }
  // tasks
  if (!communication_msgs__msg__TaskAllocation__Sequence__copy(
      &(input->tasks), &(output->tasks)))
  {
    return false;
  }
  return true;
}

communication_msgs__msg__TaskAllocationArray *
communication_msgs__msg__TaskAllocationArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  communication_msgs__msg__TaskAllocationArray * msg = (communication_msgs__msg__TaskAllocationArray *)allocator.allocate(sizeof(communication_msgs__msg__TaskAllocationArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(communication_msgs__msg__TaskAllocationArray));
  bool success = communication_msgs__msg__TaskAllocationArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
communication_msgs__msg__TaskAllocationArray__destroy(communication_msgs__msg__TaskAllocationArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    communication_msgs__msg__TaskAllocationArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
communication_msgs__msg__TaskAllocationArray__Sequence__init(communication_msgs__msg__TaskAllocationArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  communication_msgs__msg__TaskAllocationArray * data = NULL;

  if (size) {
    data = (communication_msgs__msg__TaskAllocationArray *)allocator.zero_allocate(size, sizeof(communication_msgs__msg__TaskAllocationArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = communication_msgs__msg__TaskAllocationArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        communication_msgs__msg__TaskAllocationArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
communication_msgs__msg__TaskAllocationArray__Sequence__fini(communication_msgs__msg__TaskAllocationArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      communication_msgs__msg__TaskAllocationArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

communication_msgs__msg__TaskAllocationArray__Sequence *
communication_msgs__msg__TaskAllocationArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  communication_msgs__msg__TaskAllocationArray__Sequence * array = (communication_msgs__msg__TaskAllocationArray__Sequence *)allocator.allocate(sizeof(communication_msgs__msg__TaskAllocationArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = communication_msgs__msg__TaskAllocationArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
communication_msgs__msg__TaskAllocationArray__Sequence__destroy(communication_msgs__msg__TaskAllocationArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    communication_msgs__msg__TaskAllocationArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
communication_msgs__msg__TaskAllocationArray__Sequence__are_equal(const communication_msgs__msg__TaskAllocationArray__Sequence * lhs, const communication_msgs__msg__TaskAllocationArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!communication_msgs__msg__TaskAllocationArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
communication_msgs__msg__TaskAllocationArray__Sequence__copy(
  const communication_msgs__msg__TaskAllocationArray__Sequence * input,
  communication_msgs__msg__TaskAllocationArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(communication_msgs__msg__TaskAllocationArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    communication_msgs__msg__TaskAllocationArray * data =
      (communication_msgs__msg__TaskAllocationArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!communication_msgs__msg__TaskAllocationArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          communication_msgs__msg__TaskAllocationArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!communication_msgs__msg__TaskAllocationArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
