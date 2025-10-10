// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_request_rejected
{
public:
  explicit Init_Status_request_rejected(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::Status request_rejected(::synapse_msgs::msg::Status::_request_rejected_type arg)
  {
    msg_.request_rejected = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_request_seq
{
public:
  explicit Init_Status_request_seq(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_request_rejected request_seq(::synapse_msgs::msg::Status::_request_seq_type arg)
  {
    msg_.request_seq = std::move(arg);
    return Init_Status_request_rejected(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_power
{
public:
  explicit Init_Status_power(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_request_seq power(::synapse_msgs::msg::Status::_power_type arg)
  {
    msg_.power = std::move(arg);
    return Init_Status_request_seq(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_fuel_percentage
{
public:
  explicit Init_Status_fuel_percentage(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_power fuel_percentage(::synapse_msgs::msg::Status::_fuel_percentage_type arg)
  {
    msg_.fuel_percentage = std::move(arg);
    return Init_Status_power(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_status_message
{
public:
  explicit Init_Status_status_message(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_fuel_percentage status_message(::synapse_msgs::msg::Status::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_Status_fuel_percentage(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_synapse
{
public:
  explicit Init_Status_synapse(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_status_message synapse(::synapse_msgs::msg::Status::_synapse_type arg)
  {
    msg_.synapse = std::move(arg);
    return Init_Status_status_message(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_safety
{
public:
  explicit Init_Status_safety(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_synapse safety(::synapse_msgs::msg::Status::_safety_type arg)
  {
    msg_.safety = std::move(arg);
    return Init_Status_synapse(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_mode
{
public:
  explicit Init_Status_mode(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_safety mode(::synapse_msgs::msg::Status::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Status_safety(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_fuel
{
public:
  explicit Init_Status_fuel(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_mode fuel(::synapse_msgs::msg::Status::_fuel_type arg)
  {
    msg_.fuel = std::move(arg);
    return Init_Status_mode(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_joy
{
public:
  explicit Init_Status_joy(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_fuel joy(::synapse_msgs::msg::Status::_joy_type arg)
  {
    msg_.joy = std::move(arg);
    return Init_Status_fuel(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_arming
{
public:
  explicit Init_Status_arming(::synapse_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_joy arming(::synapse_msgs::msg::Status::_arming_type arg)
  {
    msg_.arming = std::move(arg);
    return Init_Status_joy(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_arming header(::synapse_msgs::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_arming(msg_);
  }

private:
  ::synapse_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::Status>()
{
  return synapse_msgs::msg::builder::Init_Status_header();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
