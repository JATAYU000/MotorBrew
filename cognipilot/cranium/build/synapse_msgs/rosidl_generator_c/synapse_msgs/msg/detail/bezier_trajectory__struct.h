// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'curves'
#include "synapse_msgs/msg/detail/bezier_curve__struct.h"

/// Struct defined in msg/BezierTrajectory in the package synapse_msgs.
typedef struct synapse_msgs__msg__BezierTrajectory
{
  std_msgs__msg__Header header;
  uint64_t time_start;
  synapse_msgs__msg__BezierCurve__Sequence curves;
} synapse_msgs__msg__BezierTrajectory;

// Struct for a sequence of synapse_msgs__msg__BezierTrajectory.
typedef struct synapse_msgs__msg__BezierTrajectory__Sequence
{
  synapse_msgs__msg__BezierTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__BezierTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__STRUCT_H_
