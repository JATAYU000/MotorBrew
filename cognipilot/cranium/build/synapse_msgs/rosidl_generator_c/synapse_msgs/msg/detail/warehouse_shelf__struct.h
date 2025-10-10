// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__STRUCT_H_
#define SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'object_name'
// Member 'qr_decoded'
#include "rosidl_runtime_c/string.h"
// Member 'object_count'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/WarehouseShelf in the package synapse_msgs.
typedef struct synapse_msgs__msg__WarehouseShelf
{
  rosidl_runtime_c__String__Sequence object_name;
  rosidl_runtime_c__uint8__Sequence object_count;
  rosidl_runtime_c__String qr_decoded;
} synapse_msgs__msg__WarehouseShelf;

// Struct for a sequence of synapse_msgs__msg__WarehouseShelf.
typedef struct synapse_msgs__msg__WarehouseShelf__Sequence
{
  synapse_msgs__msg__WarehouseShelf * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} synapse_msgs__msg__WarehouseShelf__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__STRUCT_H_
