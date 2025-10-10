// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/BezierCurve.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'z'
// Member 'yaw'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BezierCurve in the package synapse_msgs.
typedef struct synapse_msgs__msg__BezierCurve
{
  uint64_t time_stop;
  rosidl_runtime_c__double__Sequence x;
  rosidl_runtime_c__double__Sequence y;
  rosidl_runtime_c__double__Sequence z;
  rosidl_runtime_c__double__Sequence yaw;
} synapse_msgs__msg__BezierCurve;

// Struct for a sequence of synapse_msgs__msg__BezierCurve.
typedef struct synapse_msgs__msg__BezierCurve__Sequence
{
  synapse_msgs__msg__BezierCurve * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__BezierCurve__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__STRUCT_H_
