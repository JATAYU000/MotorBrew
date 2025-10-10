// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/edge_vectors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `vector_1`
// Member `vector_2`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
synapse_msgs__msg__EdgeVectors__init(synapse_msgs__msg__EdgeVectors * msg)
{
  if (!msg) {
    return false;
  }
  // vector_1
  for (size_t i = 0; i < 2; ++i) {
    if (!geometry_msgs__msg__Point__init(&msg->vector_1[i])) {
      synapse_msgs__msg__EdgeVectors__fini(msg);
      return false;
    }
  }
  // vector_2
  for (size_t i = 0; i < 2; ++i) {
    if (!geometry_msgs__msg__Point__init(&msg->vector_2[i])) {
      synapse_msgs__msg__EdgeVectors__fini(msg);
      return false;
    }
  }
  // vector_count
  // image_height
  // image_width
  return true;
}

void
synapse_msgs__msg__EdgeVectors__fini(synapse_msgs__msg__EdgeVectors * msg)
{
  if (!msg) {
    return;
  }
  // vector_1
  for (size_t i = 0; i < 2; ++i) {
    geometry_msgs__msg__Point__fini(&msg->vector_1[i]);
  }
  // vector_2
  for (size_t i = 0; i < 2; ++i) {
    geometry_msgs__msg__Point__fini(&msg->vector_2[i]);
  }
  // vector_count
  // image_height
  // image_width
}

bool
synapse_msgs__msg__EdgeVectors__are_equal(const synapse_msgs__msg__EdgeVectors * lhs, const synapse_msgs__msg__EdgeVectors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vector_1
  for (size_t i = 0; i < 2; ++i) {
    if (!geometry_msgs__msg__Point__are_equal(
        &(lhs->vector_1[i]), &(rhs->vector_1[i])))
    {
      return false;
    }
  }
  // vector_2
  for (size_t i = 0; i < 2; ++i) {
    if (!geometry_msgs__msg__Point__are_equal(
        &(lhs->vector_2[i]), &(rhs->vector_2[i])))
    {
      return false;
    }
  }
  // vector_count
  if (lhs->vector_count != rhs->vector_count) {
    return false;
  }
  // image_height
  if (lhs->image_height != rhs->image_height) {
    return false;
  }
  // image_width
  if (lhs->image_width != rhs->image_width) {
    return false;
  }
  return true;
}

bool
synapse_msgs__msg__EdgeVectors__copy(
  const synapse_msgs__msg__EdgeVectors * input,
  synapse_msgs__msg__EdgeVectors * output)
{
  if (!input || !output) {
    return false;
  }
  // vector_1
  for (size_t i = 0; i < 2; ++i) {
    if (!geometry_msgs__msg__Point__copy(
        &(input->vector_1[i]), &(output->vector_1[i])))
    {
      return false;
    }
  }
  // vector_2
  for (size_t i = 0; i < 2; ++i) {
    if (!geometry_msgs__msg__Point__copy(
        &(input->vector_2[i]), &(output->vector_2[i])))
    {
      return false;
    }
  }
  // vector_count
  output->vector_count = input->vector_count;
  // image_height
  output->image_height = input->image_height;
  // image_width
  output->image_width = input->image_width;
  return true;
}

synapse_msgs__msg__EdgeVectors *
synapse_msgs__msg__EdgeVectors__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__EdgeVectors * msg = (synapse_msgs__msg__EdgeVectors *)allocator.allocate(sizeof(synapse_msgs__msg__EdgeVectors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(synapse_msgs__msg__EdgeVectors));
  bool success = synapse_msgs__msg__EdgeVectors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
synapse_msgs__msg__EdgeVectors__destroy(synapse_msgs__msg__EdgeVectors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    synapse_msgs__msg__EdgeVectors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
synapse_msgs__msg__EdgeVectors__Sequence__init(synapse_msgs__msg__EdgeVectors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__EdgeVectors * data = NULL;

  if (size) {
    data = (synapse_msgs__msg__EdgeVectors *)allocator.zero_allocate(size, sizeof(synapse_msgs__msg__EdgeVectors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = synapse_msgs__msg__EdgeVectors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        synapse_msgs__msg__EdgeVectors__fini(&data[i - 1]);
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
synapse_msgs__msg__EdgeVectors__Sequence__fini(synapse_msgs__msg__EdgeVectors__Sequence * array)
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
      synapse_msgs__msg__EdgeVectors__fini(&array->data[i]);
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

synapse_msgs__msg__EdgeVectors__Sequence *
synapse_msgs__msg__EdgeVectors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  synapse_msgs__msg__EdgeVectors__Sequence * array = (synapse_msgs__msg__EdgeVectors__Sequence *)allocator.allocate(sizeof(synapse_msgs__msg__EdgeVectors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = synapse_msgs__msg__EdgeVectors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
synapse_msgs__msg__EdgeVectors__Sequence__destroy(synapse_msgs__msg__EdgeVectors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    synapse_msgs__msg__EdgeVectors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
synapse_msgs__msg__EdgeVectors__Sequence__are_equal(const synapse_msgs__msg__EdgeVectors__Sequence * lhs, const synapse_msgs__msg__EdgeVectors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!synapse_msgs__msg__EdgeVectors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
synapse_msgs__msg__EdgeVectors__Sequence__copy(
  const synapse_msgs__msg__EdgeVectors__Sequence * input,
  synapse_msgs__msg__EdgeVectors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(synapse_msgs__msg__EdgeVectors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    synapse_msgs__msg__EdgeVectors * data =
      (synapse_msgs__msg__EdgeVectors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!synapse_msgs__msg__EdgeVectors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          synapse_msgs__msg__EdgeVectors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!synapse_msgs__msg__EdgeVectors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
