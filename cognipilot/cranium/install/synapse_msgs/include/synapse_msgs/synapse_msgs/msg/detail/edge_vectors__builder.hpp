// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__BUILDER_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "synapse_msgs/msg/detail/edge_vectors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace synapse_msgs
{

namespace msg
{

namespace builder
{

class Init_EdgeVectors_image_width
{
public:
  explicit Init_EdgeVectors_image_width(::synapse_msgs::msg::EdgeVectors & msg)
  : msg_(msg)
  {}
  ::synapse_msgs::msg::EdgeVectors image_width(::synapse_msgs::msg::EdgeVectors::_image_width_type arg)
  {
    msg_.image_width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::synapse_msgs::msg::EdgeVectors msg_;
};

class Init_EdgeVectors_image_height
{
public:
  explicit Init_EdgeVectors_image_height(::synapse_msgs::msg::EdgeVectors & msg)
  : msg_(msg)
  {}
  Init_EdgeVectors_image_width image_height(::synapse_msgs::msg::EdgeVectors::_image_height_type arg)
  {
    msg_.image_height = std::move(arg);
    return Init_EdgeVectors_image_width(msg_);
  }

private:
  ::synapse_msgs::msg::EdgeVectors msg_;
};

class Init_EdgeVectors_vector_count
{
public:
  explicit Init_EdgeVectors_vector_count(::synapse_msgs::msg::EdgeVectors & msg)
  : msg_(msg)
  {}
  Init_EdgeVectors_image_height vector_count(::synapse_msgs::msg::EdgeVectors::_vector_count_type arg)
  {
    msg_.vector_count = std::move(arg);
    return Init_EdgeVectors_image_height(msg_);
  }

private:
  ::synapse_msgs::msg::EdgeVectors msg_;
};

class Init_EdgeVectors_vector_2
{
public:
  explicit Init_EdgeVectors_vector_2(::synapse_msgs::msg::EdgeVectors & msg)
  : msg_(msg)
  {}
  Init_EdgeVectors_vector_count vector_2(::synapse_msgs::msg::EdgeVectors::_vector_2_type arg)
  {
    msg_.vector_2 = std::move(arg);
    return Init_EdgeVectors_vector_count(msg_);
  }

private:
  ::synapse_msgs::msg::EdgeVectors msg_;
};

class Init_EdgeVectors_vector_1
{
public:
  Init_EdgeVectors_vector_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EdgeVectors_vector_2 vector_1(::synapse_msgs::msg::EdgeVectors::_vector_1_type arg)
  {
    msg_.vector_1 = std::move(arg);
    return Init_EdgeVectors_vector_2(msg_);
  }

private:
  ::synapse_msgs::msg::EdgeVectors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::synapse_msgs::msg::EdgeVectors>()
{
  return synapse_msgs::msg::builder::Init_EdgeVectors_vector_1();
}

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__BUILDER_HPP_
