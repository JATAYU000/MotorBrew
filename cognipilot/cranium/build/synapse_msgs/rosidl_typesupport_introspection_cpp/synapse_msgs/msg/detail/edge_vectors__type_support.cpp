// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "synapse_msgs/msg/detail/edge_vectors__struct.hpp"
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

void EdgeVectors_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) synapse_msgs::msg::EdgeVectors(_init);
}

void EdgeVectors_fini_function(void * message_memory)
{
  auto typed_message = static_cast<synapse_msgs::msg::EdgeVectors *>(message_memory);
  typed_message->~EdgeVectors();
}

size_t size_function__EdgeVectors__vector_1(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__EdgeVectors__vector_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::Point, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__EdgeVectors__vector_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::Point, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__EdgeVectors__vector_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__EdgeVectors__vector_1(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__EdgeVectors__vector_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__EdgeVectors__vector_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

size_t size_function__EdgeVectors__vector_2(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__EdgeVectors__vector_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::Point, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__EdgeVectors__vector_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::Point, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__EdgeVectors__vector_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__EdgeVectors__vector_2(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__EdgeVectors__vector_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__EdgeVectors__vector_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EdgeVectors_message_member_array[5] = {
  {
    "vector_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::EdgeVectors, vector_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__EdgeVectors__vector_1,  // size() function pointer
    get_const_function__EdgeVectors__vector_1,  // get_const(index) function pointer
    get_function__EdgeVectors__vector_1,  // get(index) function pointer
    fetch_function__EdgeVectors__vector_1,  // fetch(index, &value) function pointer
    assign_function__EdgeVectors__vector_1,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vector_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::EdgeVectors, vector_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__EdgeVectors__vector_2,  // size() function pointer
    get_const_function__EdgeVectors__vector_2,  // get_const(index) function pointer
    get_function__EdgeVectors__vector_2,  // get(index) function pointer
    fetch_function__EdgeVectors__vector_2,  // fetch(index, &value) function pointer
    assign_function__EdgeVectors__vector_2,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vector_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::EdgeVectors, vector_count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "image_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::EdgeVectors, image_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "image_width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::EdgeVectors, image_width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EdgeVectors_message_members = {
  "synapse_msgs::msg",  // message namespace
  "EdgeVectors",  // message name
  5,  // number of fields
  sizeof(synapse_msgs::msg::EdgeVectors),
  EdgeVectors_message_member_array,  // message members
  EdgeVectors_init_function,  // function to initialize message memory (memory has to be allocated)
  EdgeVectors_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EdgeVectors_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EdgeVectors_message_members,
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
get_message_type_support_handle<synapse_msgs::msg::EdgeVectors>()
{
  return &::synapse_msgs::msg::rosidl_typesupport_introspection_cpp::EdgeVectors_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, synapse_msgs, msg, EdgeVectors)() {
  return &::synapse_msgs::msg::rosidl_typesupport_introspection_cpp::EdgeVectors_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
