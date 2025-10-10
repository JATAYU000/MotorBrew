// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "synapse_msgs/msg/detail/warehouse_shelf__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace synapse_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void WarehouseShelf_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) synapse_msgs::msg::WarehouseShelf(_init);
}

void WarehouseShelf_fini_function(void * message_memory)
{
  auto typed_message = static_cast<synapse_msgs::msg::WarehouseShelf *>(message_memory);
  typed_message->~WarehouseShelf();
}

size_t size_function__WarehouseShelf__object_name(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WarehouseShelf__object_name(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__WarehouseShelf__object_name(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__WarehouseShelf__object_name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__WarehouseShelf__object_name(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__WarehouseShelf__object_name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__WarehouseShelf__object_name(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__WarehouseShelf__object_name(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__WarehouseShelf__object_count(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WarehouseShelf__object_count(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__WarehouseShelf__object_count(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__WarehouseShelf__object_count(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__WarehouseShelf__object_count(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__WarehouseShelf__object_count(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__WarehouseShelf__object_count(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__WarehouseShelf__object_count(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember WarehouseShelf_message_member_array[3] = {
  {
    "object_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::WarehouseShelf, object_name),  // bytes offset in struct
    nullptr,  // default value
    size_function__WarehouseShelf__object_name,  // size() function pointer
    get_const_function__WarehouseShelf__object_name,  // get_const(index) function pointer
    get_function__WarehouseShelf__object_name,  // get(index) function pointer
    fetch_function__WarehouseShelf__object_name,  // fetch(index, &value) function pointer
    assign_function__WarehouseShelf__object_name,  // assign(index, value) function pointer
    resize_function__WarehouseShelf__object_name  // resize(index) function pointer
  },
  {
    "object_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::WarehouseShelf, object_count),  // bytes offset in struct
    nullptr,  // default value
    size_function__WarehouseShelf__object_count,  // size() function pointer
    get_const_function__WarehouseShelf__object_count,  // get_const(index) function pointer
    get_function__WarehouseShelf__object_count,  // get(index) function pointer
    fetch_function__WarehouseShelf__object_count,  // fetch(index, &value) function pointer
    assign_function__WarehouseShelf__object_count,  // assign(index, value) function pointer
    resize_function__WarehouseShelf__object_count  // resize(index) function pointer
  },
  {
    "qr_decoded",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::WarehouseShelf, qr_decoded),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers WarehouseShelf_message_members = {
  "synapse_msgs::msg",  // message namespace
  "WarehouseShelf",  // message name
  3,  // number of fields
  sizeof(synapse_msgs::msg::WarehouseShelf),
  WarehouseShelf_message_member_array,  // message members
  WarehouseShelf_init_function,  // function to initialize message memory (memory has to be allocated)
  WarehouseShelf_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t WarehouseShelf_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &WarehouseShelf_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace synapse_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<synapse_msgs::msg::WarehouseShelf>()
{
  return &::synapse_msgs::msg::rosidl_typesupport_introspection_cpp::WarehouseShelf_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, synapse_msgs, msg, WarehouseShelf)() {
  return &::synapse_msgs::msg::rosidl_typesupport_introspection_cpp::WarehouseShelf_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
