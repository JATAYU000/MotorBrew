// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/warehouse_shelf__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_WarehouseShelf_qr_decoded
{
public:
  explicit Init_WarehouseShelf_qr_decoded(::synapse_msgs::msg::WarehouseShelf & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::WarehouseShelf qr_decoded(::synapse_msgs::msg::WarehouseShelf::_qr_decoded_type arg)
  {
    msg_.qr_decoded = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::WarehouseShelf msg_;
};

class Init_WarehouseShelf_object_count
{
public:
  explicit Init_WarehouseShelf_object_count(::synapse_msgs::msg::WarehouseShelf & msg)
  : msg_(msg)
  {}
  Init_WarehouseShelf_qr_decoded object_count(::synapse_msgs::msg::WarehouseShelf::_object_count_type arg)
  {
    msg_.object_count = std::move(arg);
    return Init_WarehouseShelf_qr_decoded(msg_);
  }

private:
  ::synapse_msgs::msg::WarehouseShelf msg_;
};

class Init_WarehouseShelf_object_name
{
public:
  Init_WarehouseShelf_object_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WarehouseShelf_object_count object_name(::synapse_msgs::msg::WarehouseShelf::_object_name_type arg)
  {
    msg_.object_name = std::move(arg);
    return Init_WarehouseShelf_object_count(msg_);
  }

private:
  ::synapse_msgs::msg::WarehouseShelf msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::WarehouseShelf>()
{
  return synapse_msgs::msg::builder::Init_WarehouseShelf_object_name();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__BUILDER_HPP_
