// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/bezier_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_BezierTrajectory_curves
{
public:
  explicit Init_BezierTrajectory_curves(::synapse_msgs::msg::BezierTrajectory & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::BezierTrajectory curves(::synapse_msgs::msg::BezierTrajectory::_curves_type arg)
  {
    msg_.curves = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::BezierTrajectory msg_;
};

class Init_BezierTrajectory_time_start
{
public:
  explicit Init_BezierTrajectory_time_start(::synapse_msgs::msg::BezierTrajectory & msg)
  : msg_(msg)
  {}
  Init_BezierTrajectory_curves time_start(::synapse_msgs::msg::BezierTrajectory::_time_start_type arg)
  {
    msg_.time_start = std::move(arg);
    return Init_BezierTrajectory_curves(msg_);
  }

private:
  ::synapse_msgs::msg::BezierTrajectory msg_;
};

class Init_BezierTrajectory_header
{
public:
  Init_BezierTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BezierTrajectory_time_start header(::synapse_msgs::msg::BezierTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BezierTrajectory_time_start(msg_);
  }

private:
  ::synapse_msgs::msg::BezierTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::BezierTrajectory>()
{
  return synapse_msgs::msg::builder::Init_BezierTrajectory_header();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__BUILDER_HPP_
