// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "synapse_msgs/msg/detail/bezier_trajectory__rosidl_typesupport_introspection_c.h"
#include "synapse_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "synapse_msgs/msg/detail/bezier_trajectory__functions.h"
#include "synapse_msgs/msg/detail/bezier_trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `curves`
#include "synapse_msgs/msg/bezier_curve.h"
// Member `curves`
#include "synapse_msgs/msg/detail/bezier_curve__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  synapse_msgs__msg__BezierTrajectory__init(message_memory);
}

void synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_fini_function(void * message_memory)
{
  synapse_msgs__msg__BezierTrajectory__fini(message_memory);
}

size_t synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__size_function__BezierTrajectory__curves(
  const void * untyped_member)
{
  const synapse_msgs__msg__BezierCurve__Sequence * member =
    (const synapse_msgs__msg__BezierCurve__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__get_const_function__BezierTrajectory__curves(
  const void * untyped_member, size_t index)
{
  const synapse_msgs__msg__BezierCurve__Sequence * member =
    (const synapse_msgs__msg__BezierCurve__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__get_function__BezierTrajectory__curves(
  void * untyped_member, size_t index)
{
  synapse_msgs__msg__BezierCurve__Sequence * member =
    (synapse_msgs__msg__BezierCurve__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__fetch_function__BezierTrajectory__curves(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const synapse_msgs__msg__BezierCurve * item =
    ((const synapse_msgs__msg__BezierCurve *)
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__get_const_function__BezierTrajectory__curves(untyped_member, index));
  synapse_msgs__msg__BezierCurve * value =
    (synapse_msgs__msg__BezierCurve *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__assign_function__BezierTrajectory__curves(
  void * untyped_member, size_t index, const void * untyped_value)
{
  synapse_msgs__msg__BezierCurve * item =
    ((synapse_msgs__msg__BezierCurve *)
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__get_function__BezierTrajectory__curves(untyped_member, index));
  const synapse_msgs__msg__BezierCurve * value =
    (const synapse_msgs__msg__BezierCurve *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__resize_function__BezierTrajectory__curves(
  void * untyped_member, size_t size)
{
  synapse_msgs__msg__BezierCurve__Sequence * member =
    (synapse_msgs__msg__BezierCurve__Sequence *)(untyped_member);
  synapse_msgs__msg__BezierCurve__Sequence__fini(member);
  return synapse_msgs__msg__BezierCurve__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierTrajectory, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_start",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierTrajectory, time_start),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "curves",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__BezierTrajectory, curves),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__size_function__BezierTrajectory__curves,  // size() function pointer
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__get_const_function__BezierTrajectory__curves,  // get_const(index) function pointer
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__get_function__BezierTrajectory__curves,  // get(index) function pointer
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__fetch_function__BezierTrajectory__curves,  // fetch(index, &value) function pointer
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__assign_function__BezierTrajectory__curves,  // assign(index, value) function pointer
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__resize_function__BezierTrajectory__curves  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_members = {
  "synapse_msgs__msg",  // message namespace
  "BezierTrajectory",  // message name
  3,  // number of fields
  sizeof(synapse_msgs__msg__BezierTrajectory),
  synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_member_array,  // message members
  synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_type_support_handle = {
  0,
  &synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_synapse_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, synapse_msgs, msg, BezierTrajectory)() {
  synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, synapse_msgs, msg, BezierCurve)();
  if (!synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_type_support_handle.typesupport_identifier) {
    synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &synapse_msgs__msg__BezierTrajectory__rosidl_typesupport_introspection_c__BezierTrajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
