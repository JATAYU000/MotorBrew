// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__TRAITS_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "synapse_msgs/msg/detail/bezier_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'curves'
#include "synapse_msgs/msg/detail/bezier_curve__traits.hpp"

namespace synapse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BezierTrajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: time_start
  {
    out << "time_start: ";
    rosidl_generator_traits::value_to_yaml(msg.time_start, out);
    out << ", ";
  }

  // member: curves
  {
    if (msg.curves.size() == 0) {
      out << "curves: []";
    } else {
      out << "curves: [";
      size_t pending_items = msg.curves.size();
      for (auto item : msg.curves) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BezierTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: time_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_start: ";
    rosidl_generator_traits::value_to_yaml(msg.time_start, out);
    out << "\n";
  }

  // member: curves
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.curves.size() == 0) {
      out << "curves: []\n";
    } else {
      out << "curves:\n";
      for (auto item : msg.curves) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BezierTrajectory & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace synapse_msgs

namespace rosidl_generator_traits
{

[[deprecated("use synapse_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const synapse_msgs::msg::BezierTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  synapse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use synapse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const synapse_msgs::msg::BezierTrajectory & msg)
{
  return synapse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<synapse_msgs::msg::BezierTrajectory>()
{
  return "synapse_msgs::msg::BezierTrajectory";
}

template<>
inline const char * name<synapse_msgs::msg::BezierTrajectory>()
{
  return "synapse_msgs/msg/BezierTrajectory";
}

template<>
struct has_fixed_size<synapse_msgs::msg::BezierTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<synapse_msgs::msg::BezierTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<synapse_msgs::msg::BezierTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__TRAITS_HPP_
