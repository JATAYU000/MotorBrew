// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "synapse_msgs/msg/detail/bezier_trajectory__struct.hpp"
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

void BezierTrajectory_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) synapse_msgs::msg::BezierTrajectory(_init);
}

void BezierTrajectory_fini_function(void * message_memory)
{
  auto typed_message = static_cast<synapse_msgs::msg::BezierTrajectory *>(message_memory);
  typed_message->~BezierTrajectory();
}

size_t size_function__BezierTrajectory__curves(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<synapse_msgs::msg::BezierCurve> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BezierTrajectory__curves(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<synapse_msgs::msg::BezierCurve> *>(untyped_member);
  return &member[index];
}

void * get_function__BezierTrajectory__curves(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<synapse_msgs::msg::BezierCurve> *>(untyped_member);
  return &member[index];
}

void fetch_function__BezierTrajectory__curves(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const synapse_msgs::msg::BezierCurve *>(
    get_const_function__BezierTrajectory__curves(untyped_member, index));
  auto & value = *reinterpret_cast<synapse_msgs::msg::BezierCurve *>(untyped_value);
  value = item;
}

void assign_function__BezierTrajectory__curves(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<synapse_msgs::msg::BezierCurve *>(
    get_function__BezierTrajectory__curves(untyped_member, index));
  const auto & value = *reinterpret_cast<const synapse_msgs::msg::BezierCurve *>(untyped_value);
  item = value;
}

void resize_function__BezierTrajectory__curves(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<synapse_msgs::msg::BezierCurve> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BezierTrajectory_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::BezierTrajectory, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time_start",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::BezierTrajectory, time_start),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "curves",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<synapse_msgs::msg::BezierCurve>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(synapse_msgs::msg::BezierTrajectory, curves),  // bytes offset in struct
    nullptr,  // default value
    size_function__BezierTrajectory__curves,  // size() function pointer
    get_const_function__BezierTrajectory__curves,  // get_const(index) function pointer
    get_function__BezierTrajectory__curves,  // get(index) function pointer
    fetch_function__BezierTrajectory__curves,  // fetch(index, &value) function pointer
    assign_function__BezierTrajectory__curves,  // assign(index, value) function pointer
    resize_function__BezierTrajectory__curves  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BezierTrajectory_message_members = {
  "synapse_msgs::msg",  // message namespace
  "BezierTrajectory",  // message name
  3,  // number of fields
  sizeof(synapse_msgs::msg::BezierTrajectory),
  BezierTrajectory_message_member_array,  // message members
  BezierTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  BezierTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BezierTrajectory_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BezierTrajectory_message_members,
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
get_message_type_support_handle<synapse_msgs::msg::BezierTrajectory>()
{
  return &::synapse_msgs::msg::rosidl_typesupport_introspection_cpp::BezierTrajectory_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, synapse_msgs, msg, BezierTrajectory)() {
  return &::synapse_msgs::msg::rosidl_typesupport_introspection_cpp::BezierTrajectory_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
