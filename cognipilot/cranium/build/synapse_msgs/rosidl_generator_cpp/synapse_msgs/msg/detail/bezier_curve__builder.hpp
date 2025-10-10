// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/BezierCurve.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/bezier_curve__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_BezierCurve_yaw
{
public:
  explicit Init_BezierCurve_yaw(::synapse_msgs::msg::BezierCurve & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::BezierCurve yaw(::synapse_msgs::msg::BezierCurve::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::BezierCurve msg_;
};

class Init_BezierCurve_z
{
public:
  explicit Init_BezierCurve_z(::synapse_msgs::msg::BezierCurve & msg)
  : msg_(msg)
  {}
  Init_BezierCurve_yaw z(::synapse_msgs::msg::BezierCurve::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_BezierCurve_yaw(msg_);
  }

private:
  ::synapse_msgs::msg::BezierCurve msg_;
};

class Init_BezierCurve_y
{
public:
  explicit Init_BezierCurve_y(::synapse_msgs::msg::BezierCurve & msg)
  : msg_(msg)
  {}
  Init_BezierCurve_z y(::synapse_msgs::msg::BezierCurve::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_BezierCurve_z(msg_);
  }

private:
  ::synapse_msgs::msg::BezierCurve msg_;
};

class Init_BezierCurve_x
{
public:
  explicit Init_BezierCurve_x(::synapse_msgs::msg::BezierCurve & msg)
  : msg_(msg)
  {}
  Init_BezierCurve_y x(::synapse_msgs::msg::BezierCurve::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_BezierCurve_y(msg_);
  }

private:
  ::synapse_msgs::msg::BezierCurve msg_;
};

class Init_BezierCurve_time_stop
{
public:
  Init_BezierCurve_time_stop()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BezierCurve_x time_stop(::synapse_msgs::msg::BezierCurve::_time_stop_type arg)
  {
    msg_.time_stop = std::move(arg);
    return Init_BezierCurve_x(msg_);
  }

private:
  ::synapse_msgs::msg::BezierCurve msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::BezierCurve>()
{
  return synapse_msgs::msg::builder::Init_BezierCurve_time_stop();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__BUILDER_HPP_
