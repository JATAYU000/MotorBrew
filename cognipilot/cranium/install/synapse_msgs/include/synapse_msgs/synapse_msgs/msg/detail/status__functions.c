// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from synapse_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `status_message`
#include "rosidl_runtime_c/string_functions.h"

bool
synapse_msgs__msg__Status__init(synapse_msgs__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    synapse_msgs__msg__Status__fini(msg);
    return false;
  }
  // arming
  // joy
  // fuel
  // mode
  // safety
  // synapse
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    synapse_msgs__msg__Status__fini(msg);
    return false;
  }
  // fuel_percentage
  // power
  // request_seq
  // request_rejected
  return true;
}

void
synapse_msgs__msg__Status__fini(synapse_msgs__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // arming
  // joy
  // fuel
  // mode
  // safety
  // synapse
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
  // fuel_percentage
  // power
  // request_seq
  // request_rejected
}

bool
synapse_msgs__msg__Status__are_equal(const synapse_msgs__msg__Status * lhs, const synapse_msgs__msg__Status * rhs)
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
  // arming
  if (lhs->arming != rhs->arming) {
    return false;
  }
  // joy
  if (lhs->joy != rhs->joy) {
    return false;
  }
  // fuel
  if (lhs->fuel != rhs->fuel) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // safety
  if (lhs->safety != rhs->safety) {
    return false;
  }
  // synapse
  if (lhs->synapse != rhs->synapse) {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  // fuel_percentage
  if (lhs->fuel_percentage != rhs->fuel_percentage) {
    return false;
  }
  // power
  if (lhs->power != rhs->power) {
    return false;
  }
  // request_seq
  if (lhs->request_seq != rhs->request_seq) {
    return false;
  }
  // request_rejected
  if (lhs->request_rejected != rhs->request_rejected) {
    return false;
  }
  return true;
}

bool
synapse_msgs__msg__Status__copy(
  const synapse_msgs__msg__Status * input,
  synapse_msgs__msg__Status * output)
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
  // arming
  output->arming = input->arming;
  // joy
  output->joy = input->joy;
  // fuel
  output->fuel = input->fuel;
  // mode
  output->mode = input->mode;
  // safety
  output->safety = input->safety;
  // synapse
  output->synapse = input->synapse;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  // fuel_percentage
  output->fuel_percentage = input->fuel_percentage;
  // power
  output->power = input->power;
  // request_seq
  output->request_seq = input->request_seq;
  // request_rejected
  output->request_rejected = input->request_rejected;
  return true;
}

synapse_msgs__msg__Status *
synapse_msgs__msg__Status__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__Status * msg = (synapse_msgs__msg__Status *)allocator.allocate(sizeof(synapse_msgs__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(synapse_msgs__msg__Status));
  bool success = synapse_msgs__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
synapse_msgs__msg__Status__destroy(synapse_msgs__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    synapse_msgs__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
synapse_msgs__msg__Status__Sequence__init(synapse_msgs__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__Status * data = NULL;

  if (size) {
    data = (synapse_msgs__msg__Status *)allocator.zero_allocate(size, sizeof(synapse_msgs__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = synapse_msgs__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        synapse_msgs__msg__Status__fini(&data[i - 1]);
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
synapse_msgs__msg__Status__Sequence__fini(synapse_msgs__msg__Status__Sequence * array)
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
      synapse_msgs__msg__Status__fini(&array->data[i]);
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

synapse_msgs__msg__Status__Sequence *
synapse_msgs__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__Status__Sequence * array = (synapse_msgs__msg__Status__Sequence *)allocator.allocate(sizeof(synapse_msgs__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = synapse_msgs__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
synapse_msgs__msg__Status__Sequence__destroy(synapse_msgs__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    synapse_msgs__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
synapse_msgs__msg__Status__Sequence__are_equal(const synapse_msgs__msg__Status__Sequence * lhs, const synapse_msgs__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!synapse_msgs__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
synapse_msgs__msg__Status__Sequence__copy(
  const synapse_msgs__msg__Status__Sequence * input,
  synapse_msgs__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(synapse_msgs__msg__Status);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    synapse_msgs__msg__Status * data =
      (synapse_msgs__msg__Status *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!synapse_msgs__msg__Status__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          synapse_msgs__msg__Status__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!synapse_msgs__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
