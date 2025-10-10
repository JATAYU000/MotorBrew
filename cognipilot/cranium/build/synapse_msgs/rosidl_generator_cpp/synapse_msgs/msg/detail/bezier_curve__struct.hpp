// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from synapse_msgs:msg/BezierCurve.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__STRUCT_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__synapse_msgs__msg__BezierCurve __attribute__((deprecated))
#else
# define DEPRECATED__synapse_msgs__msg__BezierCurve __declspec(deprecated)
#endif

namespace synapse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BezierCurve_
{
  using Type = BezierCurve_<ContainerAllocator>;

  explicit BezierCurve_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_stop = 0ull;
    }
  }

  explicit BezierCurve_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_stop = 0ull;
    }
  }

  // field types and members
  using _time_stop_type =
    uint64_t;
  _time_stop_type time_stop;
  using _x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_type x;
  using _y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_type y;
  using _z_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _z_type z;
  using _yaw_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__time_stop(
    const uint64_t & _arg)
  {
    this->time_stop = _arg;
    return *this;
  }
  Type & set__x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__yaw(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    synapse_msgs::msg::BezierCurve_<ContainerAllocator> *;
  using ConstRawPtr =
    const synapse_msgs::msg::BezierCurve_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::BezierCurve_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::BezierCurve_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__synapse_msgs__msg__BezierCurve
    std::shared_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__synapse_msgs__msg__BezierCurve
    std::shared_ptr<synapse_msgs::msg::BezierCurve_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BezierCurve_ & other) const
  {
    if (this->time_stop != other.time_stop) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const BezierCurve_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BezierCurve_

// alias to use template instance with default allocator
using BezierCurve =
  synapse_msgs::msg::BezierCurve_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__BEZIER_CURVE__STRUCT_HPP_
