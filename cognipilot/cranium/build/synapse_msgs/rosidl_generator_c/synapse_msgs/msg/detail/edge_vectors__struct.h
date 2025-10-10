// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vector_1'
// Member 'vector_2'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/EdgeVectors in the package synapse_msgs.
typedef struct synapse_msgs__msg__EdgeVectors
{
  geometry_msgs__msg__Point vector_1[2];
  geometry_msgs__msg__Point vector_2[2];
  uint8_t vector_count;
  uint16_t image_height;
  uint16_t image_width;
} synapse_msgs__msg__EdgeVectors;

// Struct for a sequence of synapse_msgs__msg__EdgeVectors.
typedef struct synapse_msgs__msg__EdgeVectors__Sequence
{
  synapse_msgs__msg__EdgeVectors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__EdgeVectors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__STRUCT_H_
