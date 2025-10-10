// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ARMING_UNKNOWN'.
enum
{
  synapse_msgs__msg__Status__ARMING_UNKNOWN = 0
};

/// Constant 'ARMING_DISARMED'.
enum
{
  synapse_msgs__msg__Status__ARMING_DISARMED = 1
};

/// Constant 'ARMING_ARMED'.
enum
{
  synapse_msgs__msg__Status__ARMING_ARMED = 2
};

/// Constant 'FUEL_UNKNOWN'.
enum
{
  synapse_msgs__msg__Status__FUEL_UNKNOWN = 0
};

/// Constant 'FUEL_CRITICAL'.
enum
{
  synapse_msgs__msg__Status__FUEL_CRITICAL = 1
};

/// Constant 'FUEL_LOW'.
enum
{
  synapse_msgs__msg__Status__FUEL_LOW = 2
};

/// Constant 'FUEL_NOMINAL'.
enum
{
  synapse_msgs__msg__Status__FUEL_NOMINAL = 3
};

/// Constant 'FUEL_OVER_CAPACITY'.
enum
{
  synapse_msgs__msg__Status__FUEL_OVER_CAPACITY = 4
};

/// Constant 'JOY_UNKNOWN'.
enum
{
  synapse_msgs__msg__Status__JOY_UNKNOWN = 0
};

/// Constant 'JOY_DISABLED'.
enum
{
  synapse_msgs__msg__Status__JOY_DISABLED = 1
};

/// Constant 'JOY_LOSS'.
enum
{
  synapse_msgs__msg__Status__JOY_LOSS = 2
};

/// Constant 'JOY_NOMINAL'.
enum
{
  synapse_msgs__msg__Status__JOY_NOMINAL = 3
};

/// Constant 'MODE_UNKNOWN'.
enum
{
  synapse_msgs__msg__Status__MODE_UNKNOWN = 0
};

/// Constant 'MODE_MANUAL'.
enum
{
  synapse_msgs__msg__Status__MODE_MANUAL = 1
};

/// Constant 'MODE_AUTO'.
enum
{
  synapse_msgs__msg__Status__MODE_AUTO = 2
};

/// Constant 'MODE_CMD_VEL'.
enum
{
  synapse_msgs__msg__Status__MODE_CMD_VEL = 3
};

/// Constant 'MODE_CALIBRATION'.
enum
{
  synapse_msgs__msg__Status__MODE_CALIBRATION = 4
};

/// Constant 'SAFETY_UNKNOWN'.
enum
{
  synapse_msgs__msg__Status__SAFETY_UNKNOWN = 0
};

/// Constant 'SAFETY_SAFE'.
enum
{
  synapse_msgs__msg__Status__SAFETY_SAFE = 1
};

/// Constant 'SAFETY_UNSAFE'.
enum
{
  synapse_msgs__msg__Status__SAFETY_UNSAFE = 2
};

/// Constant 'SYNAPSE_UNKNOWN'.
enum
{
  synapse_msgs__msg__Status__SYNAPSE_UNKNOWN = 0
};

/// Constant 'SYNAPSE_DISABLED'.
enum
{
  synapse_msgs__msg__Status__SYNAPSE_DISABLED = 1
};

/// Constant 'SYNAPSE_LOSS'.
enum
{
  synapse_msgs__msg__Status__SYNAPSE_LOSS = 2
};

/// Constant 'SYNAPSE_NOMINAL'.
enum
{
  synapse_msgs__msg__Status__SYNAPSE_NOMINAL = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'status_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Status in the package synapse_msgs.
typedef struct synapse_msgs__msg__Status
{
  std_msgs__msg__Header header;
  uint8_t arming;
  uint8_t joy;
  uint8_t fuel;
  uint8_t mode;
  uint8_t safety;
  uint8_t synapse;
  rosidl_runtime_c__String status_message;
  uint8_t fuel_percentage;
  float power;
  uint32_t request_seq;
  bool request_rejected;
} synapse_msgs__msg__Status;

// Struct for a sequence of synapse_msgs__msg__Status.
typedef struct synapse_msgs__msg__Status__Sequence
{
  synapse_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
