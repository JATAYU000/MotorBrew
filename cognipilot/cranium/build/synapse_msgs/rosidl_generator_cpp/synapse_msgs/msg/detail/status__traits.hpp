// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from synapse_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "synapse_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace synapse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: arming
  {
    out << "arming: ";
    rosidl_generator_traits::value_to_yaml(msg.arming, out);
    out << ", ";
  }

  // member: joy
  {
    out << "joy: ";
    rosidl_generator_traits::value_to_yaml(msg.joy, out);
    out << ", ";
  }

  // member: fuel
  {
    out << "fuel: ";
    rosidl_generator_traits::value_to_yaml(msg.fuel, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: safety
  {
    out << "safety: ";
    rosidl_generator_traits::value_to_yaml(msg.safety, out);
    out << ", ";
  }

  // member: synapse
  {
    out << "synapse: ";
    rosidl_generator_traits::value_to_yaml(msg.synapse, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << ", ";
  }

  // member: fuel_percentage
  {
    out << "fuel_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.fuel_percentage, out);
    out << ", ";
  }

  // member: power
  {
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << ", ";
  }

  // member: request_seq
  {
    out << "request_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.request_seq, out);
    out << ", ";
  }

  // member: request_rejected
  {
    out << "request_rejected: ";
    rosidl_generator_traits::value_to_yaml(msg.request_rejected, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: arming
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arming: ";
    rosidl_generator_traits::value_to_yaml(msg.arming, out);
    out << "\n";
  }

  // member: joy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joy: ";
    rosidl_generator_traits::value_to_yaml(msg.joy, out);
    out << "\n";
  }

  // member: fuel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fuel: ";
    rosidl_generator_traits::value_to_yaml(msg.fuel, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: safety
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety: ";
    rosidl_generator_traits::value_to_yaml(msg.safety, out);
    out << "\n";
  }

  // member: synapse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "synapse: ";
    rosidl_generator_traits::value_to_yaml(msg.synapse, out);
    out << "\n";
  }

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }

  // member: fuel_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fuel_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.fuel_percentage, out);
    out << "\n";
  }

  // member: power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << "\n";
  }

  // member: request_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.request_seq, out);
    out << "\n";
  }

  // member: request_rejected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_rejected: ";
    rosidl_generator_traits::value_to_yaml(msg.request_rejected, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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
  const synapse_msgs::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  synapse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use synapse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const synapse_msgs::msg::Status & msg)
{
  return synapse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<synapse_msgs::msg::Status>()
{
  return "synapse_msgs::msg::Status";
}

template<>
inline const char * name<synapse_msgs::msg::Status>()
{
  return "synapse_msgs/msg/Status";
}

template<>
struct has_fixed_size<synapse_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<synapse_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<synapse_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYNAPSE_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
