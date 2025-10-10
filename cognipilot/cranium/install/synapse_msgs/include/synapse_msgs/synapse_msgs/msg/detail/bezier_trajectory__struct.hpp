// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from synapse_msgs:msg/BezierTrajectory.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__STRUCT_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'curves'
#include "synapse_msgs/msg/detail/bezier_curve__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__synapse_msgs__msg__BezierTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__synapse_msgs__msg__BezierTrajectory __declspec(deprecated)
#endif

namespace synapse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BezierTrajectory_
{
  using Type = BezierTrajectory_<ContainerAllocator>;

  explicit BezierTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_start = 0ull;
    }
  }

  explicit BezierTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_start = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_start_type =
    uint64_t;
  _time_start_type time_start;
  using _curves_type =
    std::vector<synapse_msgs::msg::BezierCurve_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<synapse_msgs::msg::BezierCurve_<ContainerAllocator>>>;
  _curves_type curves;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time_start(
    const uint64_t & _arg)
  {
    this->time_start = _arg;
    return *this;
  }
  Type & set__curves(
    const std::vector<synapse_msgs::msg::BezierCurve_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<synapse_msgs::msg::BezierCurve_<ContainerAllocator>>> & _arg)
  {
    this->curves = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    synapse_msgs::msg::BezierTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const synapse_msgs::msg::BezierTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::BezierTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::BezierTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__synapse_msgs__msg__BezierTrajectory
    std::shared_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__synapse_msgs__msg__BezierTrajectory
    std::shared_ptr<synapse_msgs::msg::BezierTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BezierTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_start != other.time_start) {
      return false;
    }
    if (this->curves != other.curves) {
      return false;
    }
    return true;
  }
  bool operator!=(const BezierTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BezierTrajectory_

// alias to use template instance with default allocator
using BezierTrajectory =
  synapse_msgs::msg::BezierTrajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_TRAJECTORY__STRUCT_HPP_
