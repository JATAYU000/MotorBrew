// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__STRUCT_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'vector_1'
// Member 'vector_2'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__synapse_msgs__msg__EdgeVectors __attribute__((deprecated))
#else
# define DEPRECATED__synapse_msgs__msg__EdgeVectors __declspec(deprecated)
#endif

namespace synapse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EdgeVectors_
{
  using Type = EdgeVectors_<ContainerAllocator>;

  explicit EdgeVectors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vector_1.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_init});
      this->vector_2.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_init});
      this->vector_count = 0;
      this->image_height = 0;
      this->image_width = 0;
    }
  }

  explicit EdgeVectors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vector_1(_alloc),
    vector_2(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vector_1.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_alloc, _init});
      this->vector_2.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_alloc, _init});
      this->vector_count = 0;
      this->image_height = 0;
      this->image_width = 0;
    }
  }

  // field types and members
  using _vector_1_type =
    std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 2>;
  _vector_1_type vector_1;
  using _vector_2_type =
    std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 2>;
  _vector_2_type vector_2;
  using _vector_count_type =
    uint8_t;
  _vector_count_type vector_count;
  using _image_height_type =
    uint16_t;
  _image_height_type image_height;
  using _image_width_type =
    uint16_t;
  _image_width_type image_width;

  // setters for named parameter idiom
  Type & set__vector_1(
    const std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 2> & _arg)
  {
    this->vector_1 = _arg;
    return *this;
  }
  Type & set__vector_2(
    const std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 2> & _arg)
  {
    this->vector_2 = _arg;
    return *this;
  }
  Type & set__vector_count(
    const uint8_t & _arg)
  {
    this->vector_count = _arg;
    return *this;
  }
  Type & set__image_height(
    const uint16_t & _arg)
  {
    this->image_height = _arg;
    return *this;
  }
  Type & set__image_width(
    const uint16_t & _arg)
  {
    this->image_width = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    synapse_msgs::msg::EdgeVectors_<ContainerAllocator> *;
  using ConstRawPtr =
    const synapse_msgs::msg::EdgeVectors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::EdgeVectors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::EdgeVectors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__synapse_msgs__msg__EdgeVectors
    std::shared_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__synapse_msgs__msg__EdgeVectors
    std::shared_ptr<synapse_msgs::msg::EdgeVectors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EdgeVectors_ & other) const
  {
    if (this->vector_1 != other.vector_1) {
      return false;
    }
    if (this->vector_2 != other.vector_2) {
      return false;
    }
    if (this->vector_count != other.vector_count) {
      return false;
    }
    if (this->image_height != other.image_height) {
      return false;
    }
    if (this->image_width != other.image_width) {
      return false;
    }
    return true;
  }
  bool operator!=(const EdgeVectors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EdgeVectors_

// alias to use template instance with default allocator
using EdgeVectors =
  synapse_msgs::msg::EdgeVectors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__STRUCT_HPP_
