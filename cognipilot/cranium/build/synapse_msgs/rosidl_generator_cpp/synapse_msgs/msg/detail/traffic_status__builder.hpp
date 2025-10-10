// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/traffic_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_TrafficStatus_right_sign
{
public:
  explicit Init_TrafficStatus_right_sign(::synapse_msgs::msg::TrafficStatus & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::TrafficStatus right_sign(::synapse_msgs::msg::TrafficStatus::_right_sign_type arg)
  {
    msg_.right_sign = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::TrafficStatus msg_;
};

class Init_TrafficStatus_straight_sign
{
public:
  explicit Init_TrafficStatus_straight_sign(::synapse_msgs::msg::TrafficStatus & msg)
  : msg_(msg)
  {}
  Init_TrafficStatus_right_sign straight_sign(::synapse_msgs::msg::TrafficStatus::_straight_sign_type arg)
  {
    msg_.straight_sign = std::move(arg);
    return Init_TrafficStatus_right_sign(msg_);
  }

private:
  ::synapse_msgs::msg::TrafficStatus msg_;
};

class Init_TrafficStatus_left_sign
{
public:
  explicit Init_TrafficStatus_left_sign(::synapse_msgs::msg::TrafficStatus & msg)
  : msg_(msg)
  {}
  Init_TrafficStatus_straight_sign left_sign(::synapse_msgs::msg::TrafficStatus::_left_sign_type arg)
  {
    msg_.left_sign = std::move(arg);
    return Init_TrafficStatus_straight_sign(msg_);
  }

private:
  ::synapse_msgs::msg::TrafficStatus msg_;
};

class Init_TrafficStatus_stop_sign
{
public:
  Init_TrafficStatus_stop_sign()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrafficStatus_left_sign stop_sign(::synapse_msgs::msg::TrafficStatus::_stop_sign_type arg)
  {
    msg_.stop_sign = std::move(arg);
    return Init_TrafficStatus_left_sign(msg_);
  }

private:
  ::synapse_msgs::msg::TrafficStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::TrafficStatus>()
{
  return synapse_msgs::msg::builder::Init_TrafficStatus_stop_sign();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__BUILDER_HPP_
