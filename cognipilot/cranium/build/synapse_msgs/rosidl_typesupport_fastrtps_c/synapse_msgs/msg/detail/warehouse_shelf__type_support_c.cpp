// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice
#include "synapse_msgs/msg/detail/warehouse_shelf__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "synapse_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "synapse_msgs/msg/detail/warehouse_shelf__struct.h"
#include "synapse_msgs/msg/detail/warehouse_shelf__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // object_count
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // object_count
#include "rosidl_runtime_c/string.h"  // object_name, qr_decoded
#include "rosidl_runtime_c/string_functions.h"  // object_name, qr_decoded

// forward declare type support functions


using _WarehouseShelf__ros_msg_type = synapse_msgs__msg__WarehouseShelf;

static bool _WarehouseShelf__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WarehouseShelf__ros_msg_type * ros_message = static_cast<const _WarehouseShelf__ros_msg_type *>(untyped_ros_message);
  // Field name: object_name
  {
    size_t size = ros_message->object_name.size;
    auto array_ptr = ros_message->object_name.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: object_count
  {
    size_t size = ros_message->object_count.size;
    auto array_ptr = ros_message->object_count.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: qr_decoded
  {
    const rosidl_runtime_c__String * str = &ros_message->qr_decoded;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _WarehouseShelf__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WarehouseShelf__ros_msg_type * ros_message = static_cast<_WarehouseShelf__ros_msg_type *>(untyped_ros_message);
  // Field name: object_name
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->object_name.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->object_name);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->object_name, size)) {
      fprintf(stderr, "failed to create array for field 'object_name'");
      return false;
    }
    auto array_ptr = ros_message->object_name.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'object_name'\n");
        return false;
      }
    }
  }

  // Field name: object_count
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->object_count.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->object_count);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->object_count, size)) {
      fprintf(stderr, "failed to create array for field 'object_count'");
      return false;
    }
    auto array_ptr = ros_message->object_count.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: qr_decoded
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->qr_decoded.data) {
      rosidl_runtime_c__String__init(&ros_message->qr_decoded);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->qr_decoded,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'qr_decoded'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_synapse_msgs
size_t get_serialized_size_synapse_msgs__msg__WarehouseShelf(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WarehouseShelf__ros_msg_type * ros_message = static_cast<const _WarehouseShelf__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name object_name
  {
    size_t array_size = ros_message->object_name.size;
    auto array_ptr = ros_message->object_name.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name object_count
  {
    size_t array_size = ros_message->object_count.size;
    auto array_ptr = ros_message->object_count.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name qr_decoded
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->qr_decoded.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _WarehouseShelf__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_synapse_msgs__msg__WarehouseShelf(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_synapse_msgs
size_t max_serialized_size_synapse_msgs__msg__WarehouseShelf(
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

  // member: object_name
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: object_count
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: qr_decoded
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = synapse_msgs__msg__WarehouseShelf;
    is_plain =
      (
      offsetof(DataType, qr_decoded) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _WarehouseShelf__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_synapse_msgs__msg__WarehouseShelf(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_WarehouseShelf = {
  "synapse_msgs::msg",
  "WarehouseShelf",
  _WarehouseShelf__cdr_serialize,
  _WarehouseShelf__cdr_deserialize,
  _WarehouseShelf__get_serialized_size,
  _WarehouseShelf__max_serialized_size
};

static rosidl_message_type_support_t _WarehouseShelf__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WarehouseShelf,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, synapse_msgs, msg, WarehouseShelf)() {
  return &_WarehouseShelf__type_support;
}

#if defined(__cplusplus)
}
#endif
