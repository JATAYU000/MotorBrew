// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "synapse_msgs/msg/detail/traffic_status__rosidl_typesupport_introspection_c.h"
#include "synapse_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "synapse_msgs/msg/detail/traffic_status__functions.h"
#include "synapse_msgs/msg/detail/traffic_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  synapse_msgs__msg__TrafficStatus__init(message_memory);
}

void synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_fini_function(void * message_memory)
{
  synapse_msgs__msg__TrafficStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_member_array[4] = {
  {
    "stop_sign",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__TrafficStatus, stop_sign),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_sign",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__TrafficStatus, left_sign),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "straight_sign",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__TrafficStatus, straight_sign),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_sign",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__TrafficStatus, right_sign),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_members = {
  "synapse_msgs__msg",  // message namespace
  "TrafficStatus",  // message name
  4,  // number of fields
  sizeof(synapse_msgs__msg__TrafficStatus),
  synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_member_array,  // message members
  synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_type_support_handle = {
  0,
  &synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_synapse_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, synapse_msgs, msg, TrafficStatus)() {
  if (!synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_type_support_handle.typesupport_identifier) {
    synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &synapse_msgs__msg__TrafficStatus__rosidl_typesupport_introspection_c__TrafficStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
