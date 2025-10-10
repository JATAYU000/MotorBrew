// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "synapse_msgs/msg/detail/edge_vectors__rosidl_typesupport_introspection_c.h"
#include "synapse_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "synapse_msgs/msg/detail/edge_vectors__functions.h"
#include "synapse_msgs/msg/detail/edge_vectors__struct.h"


// Include directives for member types
// Member `vector_1`
// Member `vector_2`
#include "geometry_msgs/msg/point.h"
// Member `vector_1`
// Member `vector_2`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  synapse_msgs__msg__EdgeVectors__init(message_memory);
}

void synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_fini_function(void * message_memory)
{
  synapse_msgs__msg__EdgeVectors__fini(message_memory);
}

size_t synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__size_function__EdgeVectors__vector_1(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_const_function__EdgeVectors__vector_1(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point * member =
    (const geometry_msgs__msg__Point *)(untyped_member);
  return &member[index];
}

void * synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_function__EdgeVectors__vector_1(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point * member =
    (geometry_msgs__msg__Point *)(untyped_member);
  return &member[index];
}

void synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__fetch_function__EdgeVectors__vector_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_const_function__EdgeVectors__vector_1(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__assign_function__EdgeVectors__vector_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_function__EdgeVectors__vector_1(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

size_t synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__size_function__EdgeVectors__vector_2(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_const_function__EdgeVectors__vector_2(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point * member =
    (const geometry_msgs__msg__Point *)(untyped_member);
  return &member[index];
}

void * synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_function__EdgeVectors__vector_2(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point * member =
    (geometry_msgs__msg__Point *)(untyped_member);
  return &member[index];
}

void synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__fetch_function__EdgeVectors__vector_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_const_function__EdgeVectors__vector_2(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__assign_function__EdgeVectors__vector_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_function__EdgeVectors__vector_2(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_member_array[5] = {
  {
    "vector_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__EdgeVectors, vector_1),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__size_function__EdgeVectors__vector_1,  // size() function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_const_function__EdgeVectors__vector_1,  // get_const(index) function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_function__EdgeVectors__vector_1,  // get(index) function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__fetch_function__EdgeVectors__vector_1,  // fetch(index, &value) function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__assign_function__EdgeVectors__vector_1,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vector_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__EdgeVectors, vector_2),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__size_function__EdgeVectors__vector_2,  // size() function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_const_function__EdgeVectors__vector_2,  // get_const(index) function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__get_function__EdgeVectors__vector_2,  // get(index) function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__fetch_function__EdgeVectors__vector_2,  // fetch(index, &value) function pointer
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__assign_function__EdgeVectors__vector_2,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vector_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__EdgeVectors, vector_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "image_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__EdgeVectors, image_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "image_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__EdgeVectors, image_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_members = {
  "synapse_msgs__msg",  // message namespace
  "EdgeVectors",  // message name
  5,  // number of fields
  sizeof(synapse_msgs__msg__EdgeVectors),
  synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_member_array,  // message members
  synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_init_function,  // function to initialize message memory (memory has to be allocated)
  synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_type_support_handle = {
  0,
  &synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_synapse_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, synapse_msgs, msg, EdgeVectors)() {
  synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_type_support_handle.typesupport_identifier) {
    synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &synapse_msgs__msg__EdgeVectors__rosidl_typesupport_introspection_c__EdgeVectors_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
