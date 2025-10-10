// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__TRAITS_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "synapse_msgs/msg/detail/warehouse_shelf__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace synapse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WarehouseShelf & msg,
  std::ostream & out)
{
  out << "{";
  // member: object_name
  {
    if (msg.object_name.size() == 0) {
      out << "object_name: []";
    } else {
      out << "object_name: [";
      size_t pending_items = msg.object_name.size();
      for (auto item : msg.object_name) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: object_count
  {
    if (msg.object_count.size() == 0) {
      out << "object_count: []";
    } else {
      out << "object_count: [";
      size_t pending_items = msg.object_count.size();
      for (auto item : msg.object_count) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: qr_decoded
  {
    out << "qr_decoded: ";
    rosidl_generator_traits::value_to_yaml(msg.qr_decoded, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WarehouseShelf & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: object_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.object_name.size() == 0) {
      out << "object_name: []\n";
    } else {
      out << "object_name:\n";
      for (auto item : msg.object_name) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: object_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.object_count.size() == 0) {
      out << "object_count: []\n";
    } else {
      out << "object_count:\n";
      for (auto item : msg.object_count) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: qr_decoded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qr_decoded: ";
    rosidl_generator_traits::value_to_yaml(msg.qr_decoded, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WarehouseShelf & msg, bool use_flow_style = false)
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
  const synapse_msgs::msg::WarehouseShelf & msg,
  std::ostream & out, size_t indentation = 0)
{
  synapse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use synapse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const synapse_msgs::msg::WarehouseShelf & msg)
{
  return synapse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<synapse_msgs::msg::WarehouseShelf>()
{
  return "synapse_msgs::msg::WarehouseShelf";
}

template<>
inline const char * name<synapse_msgs::msg::WarehouseShelf>()
{
  return "synapse_msgs/msg/WarehouseShelf";
}

template<>
struct has_fixed_size<synapse_msgs::msg::WarehouseShelf>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<synapse_msgs::msg::WarehouseShelf>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<synapse_msgs::msg::WarehouseShelf>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__TRAITS_HPP_
