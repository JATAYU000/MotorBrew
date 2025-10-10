// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__TRAITS_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "synapse_msgs/msg/detail/traffic_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace synapse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrafficStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: stop_sign
  {
    out << "stop_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_sign, out);
    out << ", ";
  }

  // member: left_sign
  {
    out << "left_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.left_sign, out);
    out << ", ";
  }

  // member: straight_sign
  {
    out << "straight_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.straight_sign, out);
    out << ", ";
  }

  // member: right_sign
  {
    out << "right_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.right_sign, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrafficStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stop_sign
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_sign, out);
    out << "\n";
  }

  // member: left_sign
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.left_sign, out);
    out << "\n";
  }

  // member: straight_sign
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "straight_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.straight_sign, out);
    out << "\n";
  }

  // member: right_sign
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_sign: ";
    rosidl_generator_traits::value_to_yaml(msg.right_sign, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrafficStatus & msg, bool use_flow_style = false)
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
  const synapse_msgs::msg::TrafficStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  synapse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use synapse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const synapse_msgs::msg::TrafficStatus & msg)
{
  return synapse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<synapse_msgs::msg::TrafficStatus>()
{
  return "synapse_msgs::msg::TrafficStatus";
}

template<>
inline const char * name<synapse_msgs::msg::TrafficStatus>()
{
  return "synapse_msgs/msg/TrafficStatus";
}

template<>
struct has_fixed_size<synapse_msgs::msg::TrafficStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<synapse_msgs::msg::TrafficStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<synapse_msgs::msg::TrafficStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__TRAITS_HPP_
