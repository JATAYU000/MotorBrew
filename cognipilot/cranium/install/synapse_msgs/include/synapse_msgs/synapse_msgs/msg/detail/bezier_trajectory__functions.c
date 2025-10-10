// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/bezier_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `curves`
#include "synapse_msgs/msg/detail/bezier_curve__functions.h"

bool
synapse_msgs__msg__BezierTrajectory__init(synapse_msgs__msg__BezierTrajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    synapse_msgs__msg__BezierTrajectory__fini(msg);
    return false;
  }
  // time_start
  // curves
  if (!synapse_msgs__msg__BezierCurve__Sequence__init(&msg->curves, 0)) {
    synapse_msgs__msg__BezierTrajectory__fini(msg);
    return false;
  }
  return true;
}

void
synapse_msgs__msg__BezierTrajectory__fini(synapse_msgs__msg__BezierTrajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // time_start
  // curves
  synapse_msgs__msg__BezierCurve__Sequence__fini(&msg->curves);
}

bool
synapse_msgs__msg__BezierTrajectory__are_equal(const synapse_msgs__msg__BezierTrajectory * lhs, const synapse_msgs__msg__BezierTrajectory * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // time_start
  if (lhs->time_start != rhs->time_start) {
    return false;
  }
  // curves
  if (!synapse_msgs__msg__BezierCurve__Sequence__are_equal(
      &(lhs->curves), &(rhs->curves)))
  {
    return false;
  }
  return true;
}

bool
synapse_msgs__msg__BezierTrajectory__copy(
  const synapse_msgs__msg__BezierTrajectory * input,
  synapse_msgs__msg__BezierTrajectory * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // time_start
  output->time_start = input->time_start;
  // curves
  if (!synapse_msgs__msg__BezierCurve__Sequence__copy(
      &(input->curves), &(output->curves)))
  {
    return false;
  }
  return true;
}

synapse_msgs__msg__BezierTrajectory *
synapse_msgs__msg__BezierTrajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__BezierTrajectory * msg = (synapse_msgs__msg__BezierTrajectory *)allocator.allocate(sizeof(synapse_msgs__msg__BezierTrajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(synapse_msgs__msg__BezierTrajectory));
  bool success = synapse_msgs__msg__BezierTrajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
synapse_msgs__msg__BezierTrajectory__destroy(synapse_msgs__msg__BezierTrajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    synapse_msgs__msg__BezierTrajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
synapse_msgs__msg__BezierTrajectory__Sequence__init(synapse_msgs__msg__BezierTrajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__BezierTrajectory * data = NULL;

  if (size) {
    data = (synapse_msgs__msg__BezierTrajectory *)allocator.zero_allocate(size, sizeof(synapse_msgs__msg__BezierTrajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = synapse_msgs__msg__BezierTrajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        synapse_msgs__msg__BezierTrajectory__fini(&data[i - 1]);
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
synapse_msgs__msg__BezierTrajectory__Sequence__fini(synapse_msgs__msg__BezierTrajectory__Sequence * array)
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
      synapse_msgs__msg__BezierTrajectory__fini(&array->data[i]);
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

synapse_msgs__msg__BezierTrajectory__Sequence *
synapse_msgs__msg__BezierTrajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__BezierTrajectory__Sequence * array = (synapse_msgs__msg__BezierTrajectory__Sequence *)allocator.allocate(sizeof(synapse_msgs__msg__BezierTrajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = synapse_msgs__msg__BezierTrajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
synapse_msgs__msg__BezierTrajectory__Sequence__destroy(synapse_msgs__msg__BezierTrajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    synapse_msgs__msg__BezierTrajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
synapse_msgs__msg__BezierTrajectory__Sequence__are_equal(const synapse_msgs__msg__BezierTrajectory__Sequence * lhs, const synapse_msgs__msg__BezierTrajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!synapse_msgs__msg__BezierTrajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
synapse_msgs__msg__BezierTrajectory__Sequence__copy(
  const synapse_msgs__msg__BezierTrajectory__Sequence * input,
  synapse_msgs__msg__BezierTrajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(synapse_msgs__msg__BezierTrajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    synapse_msgs__msg__BezierTrajectory * data =
      (synapse_msgs__msg__BezierTrajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!synapse_msgs__msg__BezierTrajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          synapse_msgs__msg__BezierTrajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!synapse_msgs__msg__BezierTrajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
