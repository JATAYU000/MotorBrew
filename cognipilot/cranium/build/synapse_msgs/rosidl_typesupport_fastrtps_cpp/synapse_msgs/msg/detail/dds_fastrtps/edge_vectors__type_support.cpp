// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/edge_vectors__rosidl_typesupport_fastrtps_cpp.hpp"
#include "synapse_msgs/msg/detail/edge_vectors__struct.hpp"

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
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Point &);
size_t get_serialized_size(
  const geometry_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

// functions for geometry_msgs::msg::Point already declared above


namespace synapse_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
cdr_serialize(
  const synapse_msgs::msg::EdgeVectors & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: vector_1
  {
    for (size_t i = 0; i < 2; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.vector_1[i],
        cdr);
    }
  }
  // Member: vector_2
  {
    for (size_t i = 0; i < 2; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.vector_2[i],
        cdr);
    }
  }
  // Member: vector_count
  cdr << ros_message.vector_count;
  // Member: image_height
  cdr << ros_message.image_height;
  // Member: image_width
  cdr << ros_message.image_width;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  synapse_msgs::msg::EdgeVectors & ros_message)
{
  // Member: vector_1
  {
    for (size_t i = 0; i < 2; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.vector_1[i]);
    }
  }

  // Member: vector_2
  {
    for (size_t i = 0; i < 2; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.vector_2[i]);
    }
  }

  // Member: vector_count
  cdr >> ros_message.vector_count;

  // Member: image_height
  cdr >> ros_message.image_height;

  // Member: image_width
  cdr >> ros_message.image_width;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
get_serialized_size(
  const synapse_msgs::msg::EdgeVectors & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: vector_1
  {
    size_t array_size = 2;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.vector_1[index], current_alignment);
    }
  }
  // Member: vector_2
  {
    size_t array_size = 2;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.vector_2[index], current_alignment);
    }
  }
  // Member: vector_count
  {
    size_t item_size = sizeof(ros_message.vector_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: image_height
  {
    size_t item_size = sizeof(ros_message.image_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: image_width
  {
    size_t item_size = sizeof(ros_message.image_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_synapse_msgs
max_serialized_size_EdgeVectors(
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


  // Member: vector_1
  {
    size_t array_size = 2;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: vector_2
  {
    size_t array_size = 2;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: vector_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: image_height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: image_width
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = synapse_msgs::msg::EdgeVectors;
    is_plain =
      (
      offsetof(DataType, image_width) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _EdgeVectors__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const synapse_msgs::msg::EdgeVectors *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EdgeVectors__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<synapse_msgs::msg::EdgeVectors *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EdgeVectors__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const synapse_msgs::msg::EdgeVectors *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EdgeVectors__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EdgeVectors(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EdgeVectors__callbacks = {
  "synapse_msgs::msg",
  "EdgeVectors",
  _EdgeVectors__cdr_serialize,
  _EdgeVectors__cdr_deserialize,
  _EdgeVectors__get_serialized_size,
  _EdgeVectors__max_serialized_size
};

static rosidl_message_type_support_t _EdgeVectors__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EdgeVectors__callbacks,
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
get_message_type_support_handle<synapse_msgs::msg::EdgeVectors>()
{
  return &synapse_msgs::msg::typesupport_fastrtps_cpp::_EdgeVectors__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, synapse_msgs, msg, EdgeVectors)() {
  return &synapse_msgs::msg::typesupport_fastrtps_cpp::_EdgeVectors__handle;
}

#ifdef __cplusplus
}
#endif
