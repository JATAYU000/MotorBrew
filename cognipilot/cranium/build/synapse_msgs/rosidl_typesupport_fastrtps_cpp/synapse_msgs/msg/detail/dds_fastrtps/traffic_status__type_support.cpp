// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/traffic_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "synapse_msgs/msg/detail/traffic_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace synapse_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
cdr_serialize(
  const synapse_msgs::msg::TrafficStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: stop_sign
  cdr << (ros_message.stop_sign ? true : false);
  // Member: left_sign
  cdr << (ros_message.left_sign ? true : false);
  // Member: straight_sign
  cdr << (ros_message.straight_sign ? true : false);
  // Member: right_sign
  cdr << (ros_message.right_sign ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  synapse_msgs::msg::TrafficStatus & ros_message)
{
  // Member: stop_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.stop_sign = tmp ? true : false;
  }

  // Member: left_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.left_sign = tmp ? true : false;
  }

  // Member: straight_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.straight_sign = tmp ? true : false;
  }

  // Member: right_sign
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.right_sign = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
get_serialized_size(
  const synapse_msgs::msg::TrafficStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: stop_sign
  {
    size_t item_size = sizeof(ros_message.stop_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_sign
  {
    size_t item_size = sizeof(ros_message.left_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: straight_sign
  {
    size_t item_size = sizeof(ros_message.straight_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_sign
  {
    size_t item_size = sizeof(ros_message.right_sign);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
max_serialized_size_TrafficStatus(
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


  // Member: stop_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: left_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: straight_sign
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: right_sign
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
    using DataType = synapse_msgs::msg::TrafficStatus;
    is_plain =
      (
      offsetof(DataType, right_sign) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _TrafficStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const synapse_msgs::msg::TrafficStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TrafficStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<synapse_msgs::msg::TrafficStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TrafficStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const synapse_msgs::msg::TrafficStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TrafficStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TrafficStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TrafficStatus__callbacks = {
  "synapse_msgs::msg",
  "TrafficStatus",
  _TrafficStatus__cdr_serialize,
  _TrafficStatus__cdr_deserialize,
  _TrafficStatus__get_serialized_size,
  _TrafficStatus__max_serialized_size
};

static rosidl_message_type_support_t _TrafficStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TrafficStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace synapse_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_synapse_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<synapse_msgs::msg::TrafficStatus>()
{
  return &synapse_msgs::msg::typesupport_fastrtps_cpp::_TrafficStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, synapse_msgs, msg, TrafficStatus)() {
  return &synapse_msgs::msg::typesupport_fastrtps_cpp::_TrafficStatus__handle;
}

#ifdef __cplusplus
}
#endif
