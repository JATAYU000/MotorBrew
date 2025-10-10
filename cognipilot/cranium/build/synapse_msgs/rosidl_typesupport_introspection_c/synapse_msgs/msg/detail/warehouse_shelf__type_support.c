// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "synapse_msgs/msg/detail/warehouse_shelf__rosidl_typesupport_introspection_c.h"
#include "synapse_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "synapse_msgs/msg/detail/warehouse_shelf__functions.h"
#include "synapse_msgs/msg/detail/warehouse_shelf__struct.h"


// Include directives for member types
// Member `object_name`
// Member `qr_decoded`
#include "rosidl_runtime_c/string_functions.h"
// Member `object_count`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  synapse_msgs__msg__WarehouseShelf__init(message_memory);
}

void synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_fini_function(void * message_memory)
{
  synapse_msgs__msg__WarehouseShelf__fini(message_memory);
}

size_t synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__size_function__WarehouseShelf__object_name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_const_function__WarehouseShelf__object_name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_function__WarehouseShelf__object_name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__fetch_function__WarehouseShelf__object_name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_const_function__WarehouseShelf__object_name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__assign_function__WarehouseShelf__object_name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_function__WarehouseShelf__object_name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__resize_function__WarehouseShelf__object_name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__size_function__WarehouseShelf__object_count(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_const_function__WarehouseShelf__object_count(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_function__WarehouseShelf__object_count(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__fetch_function__WarehouseShelf__object_count(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_const_function__WarehouseShelf__object_count(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__assign_function__WarehouseShelf__object_count(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_function__WarehouseShelf__object_count(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__resize_function__WarehouseShelf__object_count(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_member_array[3] = {
  {
    "object_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__WarehouseShelf, object_name),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__size_function__WarehouseShelf__object_name,  // size() function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_const_function__WarehouseShelf__object_name,  // get_const(index) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_function__WarehouseShelf__object_name,  // get(index) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__fetch_function__WarehouseShelf__object_name,  // fetch(index, &value) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__assign_function__WarehouseShelf__object_name,  // assign(index, value) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__resize_function__WarehouseShelf__object_name  // resize(index) function pointer
  },
  {
    "object_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__WarehouseShelf, object_count),  // bytes offset in struct
    NULL,  // default value
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__size_function__WarehouseShelf__object_count,  // size() function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_const_function__WarehouseShelf__object_count,  // get_const(index) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__get_function__WarehouseShelf__object_count,  // get(index) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__fetch_function__WarehouseShelf__object_count,  // fetch(index, &value) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__assign_function__WarehouseShelf__object_count,  // assign(index, value) function pointer
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__resize_function__WarehouseShelf__object_count  // resize(index) function pointer
  },
  {
    "qr_decoded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs__msg__WarehouseShelf, qr_decoded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_members = {
  "synapse_msgs__msg",  // message namespace
  "WarehouseShelf",  // message name
  3,  // number of fields
  sizeof(synapse_msgs__msg__WarehouseShelf),
  synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_member_array,  // message members
  synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_init_function,  // function to initialize message memory (memory has to be allocated)
  synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_type_support_handle = {
  0,
  &synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_synapse_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, synapse_msgs, msg, WarehouseShelf)() {
  if (!synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_type_support_handle.typesupport_identifier) {
    synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &synapse_msgs__msg__WarehouseShelf__rosidl_typesupport_introspection_c__WarehouseShelf_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
