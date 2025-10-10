// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/traffic_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
synapse_msgs__msg__TrafficStatus__init(synapse_msgs__msg__TrafficStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stop_sign
  // left_sign
  // straight_sign
  // right_sign
  return true;
}

void
synapse_msgs__msg__TrafficStatus__fini(synapse_msgs__msg__TrafficStatus * msg)
{
  if (!msg) {
    return;
  }
  // stop_sign
  // left_sign
  // straight_sign
  // right_sign
}

bool
synapse_msgs__msg__TrafficStatus__are_equal(const synapse_msgs__msg__TrafficStatus * lhs, const synapse_msgs__msg__TrafficStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stop_sign
  if (lhs->stop_sign != rhs->stop_sign) {
    return false;
  }
  // left_sign
  if (lhs->left_sign != rhs->left_sign) {
    return false;
  }
  // straight_sign
  if (lhs->straight_sign != rhs->straight_sign) {
    return false;
  }
  // right_sign
  if (lhs->right_sign != rhs->right_sign) {
    return false;
  }
  return true;
}

bool
synapse_msgs__msg__TrafficStatus__copy(
  const synapse_msgs__msg__TrafficStatus * input,
  synapse_msgs__msg__TrafficStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // stop_sign
  output->stop_sign = input->stop_sign;
  // left_sign
  output->left_sign = input->left_sign;
  // straight_sign
  output->straight_sign = input->straight_sign;
  // right_sign
  output->right_sign = input->right_sign;
  return true;
}

synapse_msgs__msg__TrafficStatus *
synapse_msgs__msg__TrafficStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__TrafficStatus * msg = (synapse_msgs__msg__TrafficStatus *)allocator.allocate(sizeof(synapse_msgs__msg__TrafficStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(synapse_msgs__msg__TrafficStatus));
  bool success = synapse_msgs__msg__TrafficStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
synapse_msgs__msg__TrafficStatus__destroy(synapse_msgs__msg__TrafficStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    synapse_msgs__msg__TrafficStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
synapse_msgs__msg__TrafficStatus__Sequence__init(synapse_msgs__msg__TrafficStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__TrafficStatus * data = NULL;

  if (size) {
    data = (synapse_msgs__msg__TrafficStatus *)allocator.zero_allocate(size, sizeof(synapse_msgs__msg__TrafficStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = synapse_msgs__msg__TrafficStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        synapse_msgs__msg__TrafficStatus__fini(&data[i - 1]);
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
synapse_msgs__msg__TrafficStatus__Sequence__fini(synapse_msgs__msg__TrafficStatus__Sequence * array)
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
      synapse_msgs__msg__TrafficStatus__fini(&array->data[i]);
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

synapse_msgs__msg__TrafficStatus__Sequence *
synapse_msgs__msg__TrafficStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__TrafficStatus__Sequence * array = (synapse_msgs__msg__TrafficStatus__Sequence *)allocator.allocate(sizeof(synapse_msgs__msg__TrafficStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = synapse_msgs__msg__TrafficStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
synapse_msgs__msg__TrafficStatus__Sequence__destroy(synapse_msgs__msg__TrafficStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    synapse_msgs__msg__TrafficStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
synapse_msgs__msg__TrafficStatus__Sequence__are_equal(const synapse_msgs__msg__TrafficStatus__Sequence * lhs, const synapse_msgs__msg__TrafficStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!synapse_msgs__msg__TrafficStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
synapse_msgs__msg__TrafficStatus__Sequence__copy(
  const synapse_msgs__msg__TrafficStatus__Sequence * input,
  synapse_msgs__msg__TrafficStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(synapse_msgs__msg__TrafficStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    synapse_msgs__msg__TrafficStatus * data =
      (synapse_msgs__msg__TrafficStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!synapse_msgs__msg__TrafficStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          synapse_msgs__msg__TrafficStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!synapse_msgs__msg__TrafficStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
