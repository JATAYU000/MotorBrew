// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/traffic_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "synapse_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "synapse_msgs/msg/detail/traffic_status__struct.h"
#include "synapse_msgs/msg/detail/traffic_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TrafficStatus__ros_msg_type = synapse_msgs__msg__TrafficStatus;

static bool _TrafficStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TrafficStatus__ros_msg_type * ros_message = static_cast<const _TrafficStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: stop_sign
  {
    cdr << (ros_message->stop_sign ? true : false);
  }

  // Field name: left_sign
  {
    cdr << (ros_message->left_sign ? true : false);
  }

  // Field name: straight_sign
  {
    cdr << (ros_message->straight_sign ? true : false);
  }

  // Field name: right_sign
  {
    cdr << (ros_message->right_sign ? true : false);
  }

  return true;
}

static bool _TrafficStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TrafficStatus__ros_msg_type * ros_message = static_cast<_TrafficStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: stop_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stop_sign = tmp ? true : false;
  }

  // Field name: left_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->left_sign = tmp ? true : false;
  }

  // Field name: straight_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->straight_sign = tmp ? true : false;
  }

  // Field name: right_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->right_sign = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_synapse_msgs
size_t get_serialized_size_synapse_msgs__msg__TrafficStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TrafficStatus__ros_msg_type * ros_message = static_cast<const _TrafficStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stop_sign
  {
    size_t item_size = sizeof(ros_message->stop_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_sign
  {
    size_t item_size = sizeof(ros_message->left_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name straight_sign
  {
    size_t item_size = sizeof(ros_message->straight_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_sign
  {
    size_t item_size = sizeof(ros_message->right_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TrafficStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_synapse_msgs__msg__TrafficStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_synapse_msgs
size_t max_serialized_size_synapse_msgs__msg__TrafficStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: stop_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: left_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: straight_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: right_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = synapse_msgs__msg__TrafficStatus;
    is_plain =
      (
      offsetof(DataType, right_sign) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TrafficStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_synapse_msgs__msg__TrafficStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TrafficStatus = {
  "synapse_msgs::msg",
  "TrafficStatus",
  _TrafficStatus__cdr_serialize,
  _TrafficStatus__cdr_deserialize,
  _TrafficStatus__get_serialized_size,
  _TrafficStatus__max_serialized_size
};

static rosidl_message_type_support_t _TrafficStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TrafficStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, synapse_msgs, msg, TrafficStatus)() {
  return &_TrafficStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
