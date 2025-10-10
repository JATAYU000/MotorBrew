// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from synapse_msgs:msg/BezierCurve.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "synapse_msgs/msg/detail/bezier_curve__rosidl_typesupport_introspection_c.h"
#include "synapse_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "synapse_msgs/msg/detail/bezier_curve__functions.h"
#include "synapse_msgs/msg/detail/bezier_curve__struct.h"


// Include directives for member types
// Member `x`
// Member `y`
// Member `z`
// Member `yaw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  synapse_msgs__msg__BezierCurve__init(message_memory);
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_fini_function(void * message_memory)
{
  synapse_msgs__msg__BezierCurve__fini(message_memory);
}

size_t synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__z(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__z(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__z(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__yaw(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__yaw(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__yaw(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__yaw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__yaw(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__yaw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__yaw(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__yaw(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_member_array[5] = {
  {
    "time_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierCurve, time_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierCurve, x),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__x,  // size() function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__x,  // get_const(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__x,  // get(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__x,  // fetch(index, &value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__x,  // assign(index, value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__x  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierCurve, y),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__y,  // size() function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__y,  // get_const(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__y,  // get(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__y,  // fetch(index, &value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__y,  // assign(index, value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__y  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierCurve, z),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__z,  // size() function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__z,  // get_const(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__z,  // get(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__z,  // fetch(index, &value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__z,  // assign(index, value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__z  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierCurve, yaw),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__size_function__BezierCurve__yaw,  // size() function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_const_function__BezierCurve__yaw,  // get_const(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__get_function__BezierCurve__yaw,  // get(index) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__fetch_function__BezierCurve__yaw,  // fetch(index, &value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__assign_function__BezierCurve__yaw,  // assign(index, value) function pointer
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__resize_function__BezierCurve__yaw  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_members = {
  "synapse_msgs__msg",  // message namespace
  "BezierCurve",  // message name
  5,  // number of fields
  sizeof(synapse_msgs__msg__BezierCurve),
  synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_member_array,  // message members
  synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_init_function,  // function to initialize message memory (memory has to be allocated)
  synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_type_support_handle = {
  0,
  &synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_synapse_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, synapse_msgs, msg, BezierCurve)() {
  if (!synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_type_support_handle.typesupport_identifier) {
    synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &synapse_msgs__msg__BezierCurve__rosidl_typesupport_introspection_c__BezierCurve_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
