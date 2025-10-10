// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from synapse_msgs:msg/BezierCurve.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/bezier_curve__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `x`
// Member `y`
// Member `z`
// Member `yaw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
synapse_msgs__msg__BezierCurve__init(synapse_msgs__msg__BezierCurve * msg)
{
  if (!msg) {
    return false;
  }
  // time_stop
  // x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x, 0)) {
    synapse_msgs__msg__BezierCurve__fini(msg);
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y, 0)) {
    synapse_msgs__msg__BezierCurve__fini(msg);
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z, 0)) {
    synapse_msgs__msg__BezierCurve__fini(msg);
    return false;
  }
  // yaw
  if (!rosidl_runtime_c__double__Sequence__init(&msg->yaw, 0)) {
    synapse_msgs__msg__BezierCurve__fini(msg);
    return false;
  }
  return true;
}

void
synapse_msgs__msg__BezierCurve__fini(synapse_msgs__msg__BezierCurve * msg)
{
  if (!msg) {
    return;
  }
  // time_stop
  // x
  rosidl_runtime_c__double__Sequence__fini(&msg->x);
  // y
  rosidl_runtime_c__double__Sequence__fini(&msg->y);
  // z
  rosidl_runtime_c__double__Sequence__fini(&msg->z);
  // yaw
  rosidl_runtime_c__double__Sequence__fini(&msg->yaw);
}

bool
synapse_msgs__msg__BezierCurve__are_equal(const synapse_msgs__msg__BezierCurve * lhs, const synapse_msgs__msg__BezierCurve * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // time_stop
  if (lhs->time_stop != rhs->time_stop) {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x), &(rhs->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y), &(rhs->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->z), &(rhs->z)))
  {
    return false;
  }
  // yaw
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->yaw), &(rhs->yaw)))
  {
    return false;
  }
  return true;
}

bool
synapse_msgs__msg__BezierCurve__copy(
  const synapse_msgs__msg__BezierCurve * input,
  synapse_msgs__msg__BezierCurve * output)
{
  if (!input || !output) {
    return false;
  }
  // time_stop
  output->time_stop = input->time_stop;
  // x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x), &(output->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y), &(output->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->z), &(output->z)))
  {
    return false;
  }
  // yaw
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->yaw), &(output->yaw)))
  {
    return false;
  }
  return true;
}

synapse_msgs__msg__BezierCurve *
synapse_msgs__msg__BezierCurve__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__BezierCurve * msg = (synapse_msgs__msg__BezierCurve *)allocator.allocate(sizeof(synapse_msgs__msg__BezierCurve), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(synapse_msgs__msg__BezierCurve));
  bool success = synapse_msgs__msg__BezierCurve__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
synapse_msgs__msg__BezierCurve__destroy(synapse_msgs__msg__BezierCurve * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    synapse_msgs__msg__BezierCurve__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
synapse_msgs__msg__BezierCurve__Sequence__init(synapse_msgs__msg__BezierCurve__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__BezierCurve * data = NULL;

  if (size) {
    data = (synapse_msgs__msg__BezierCurve *)allocator.zero_allocate(size, sizeof(synapse_msgs__msg__BezierCurve), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = synapse_msgs__msg__BezierCurve__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        synapse_msgs__msg__BezierCurve__fini(&data[i - 1]);
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
synapse_msgs__msg__BezierCurve__Sequence__fini(synapse_msgs__msg__BezierCurve__Sequence * array)
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
      synapse_msgs__msg__BezierCurve__fini(&array->data[i]);
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

synapse_msgs__msg__BezierCurve__Sequence *
synapse_msgs__msg__BezierCurve__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__BezierCurve__Sequence * array = (synapse_msgs__msg__BezierCurve__Sequence *)allocator.allocate(sizeof(synapse_msgs__msg__BezierCurve__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = synapse_msgs__msg__BezierCurve__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
synapse_msgs__msg__BezierCurve__Sequence__destroy(synapse_msgs__msg__BezierCurve__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    synapse_msgs__msg__BezierCurve__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
synapse_msgs__msg__BezierCurve__Sequence__are_equal(const synapse_msgs__msg__BezierCurve__Sequence * lhs, const synapse_msgs__msg__BezierCurve__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!synapse_msgs__msg__BezierCurve__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
synapse_msgs__msg__BezierCurve__Sequence__copy(
  const synapse_msgs__msg__BezierCurve__Sequence * input,
  synapse_msgs__msg__BezierCurve__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(synapse_msgs__msg__BezierCurve);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    synapse_msgs__msg__BezierCurve * data =
      (synapse_msgs__msg__BezierCurve *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!synapse_msgs__msg__BezierCurve__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          synapse_msgs__msg__BezierCurve__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!synapse_msgs__msg__BezierCurve__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
