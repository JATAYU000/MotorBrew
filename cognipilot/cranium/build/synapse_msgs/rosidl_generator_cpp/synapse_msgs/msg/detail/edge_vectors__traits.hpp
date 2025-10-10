// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__TRAITS_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "synapse_msgs/msg/detail/edge_vectors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vector_1'
// Member 'vector_2'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace synapse_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EdgeVectors & msg,
  std::ostream & out)
{
  out << "{";
  // member: vector_1
  {
    if (msg.vector_1.size() == 0) {
      out << "vector_1: []";
    } else {
      out << "vector_1: [";
      size_t pending_items = msg.vector_1.size();
      for (auto item : msg.vector_1) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vector_2
  {
    if (msg.vector_2.size() == 0) {
      out << "vector_2: []";
    } else {
      out << "vector_2: [";
      size_t pending_items = msg.vector_2.size();
      for (auto item : msg.vector_2) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vector_count
  {
    out << "vector_count: ";
    rosidl_generator_traits::value_to_yaml(msg.vector_count, out);
    out << ", ";
  }

  // member: image_height
  {
    out << "image_height: ";
    rosidl_generator_traits::value_to_yaml(msg.image_height, out);
    out << ", ";
  }

  // member: image_width
  {
    out << "image_width: ";
    rosidl_generator_traits::value_to_yaml(msg.image_width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EdgeVectors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vector_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vector_1.size() == 0) {
      out << "vector_1: []\n";
    } else {
      out << "vector_1:\n";
      for (auto item : msg.vector_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: vector_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vector_2.size() == 0) {
      out << "vector_2: []\n";
    } else {
      out << "vector_2:\n";
      for (auto item : msg.vector_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: vector_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vector_count: ";
    rosidl_generator_traits::value_to_yaml(msg.vector_count, out);
    out << "\n";
  }

  // member: image_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image_height: ";
    rosidl_generator_traits::value_to_yaml(msg.image_height, out);
    out << "\n";
  }

  // member: image_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image_width: ";
    rosidl_generator_traits::value_to_yaml(msg.image_width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EdgeVectors & msg, bool use_flow_style = false)
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
  const synapse_msgs::msg::EdgeVectors & msg,
  std::ostream & out, size_t indentation = 0)
{
  synapse_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use synapse_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const synapse_msgs::msg::EdgeVectors & msg)
{
  return synapse_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<synapse_msgs::msg::EdgeVectors>()
{
  return "synapse_msgs::msg::EdgeVectors";
}

template<>
inline const char * name<synapse_msgs::msg::EdgeVectors>()
{
  return "synapse_msgs/msg/EdgeVectors";
}

template<>
struct has_fixed_size<synapse_msgs::msg::EdgeVectors>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<synapse_msgs::msg::EdgeVectors>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<synapse_msgs::msg::EdgeVectors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__TRAITS_HPP_
