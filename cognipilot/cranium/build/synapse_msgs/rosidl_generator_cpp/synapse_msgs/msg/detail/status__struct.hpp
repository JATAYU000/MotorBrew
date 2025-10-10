// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from synapse_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__synapse_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__synapse_msgs__msg__Status __declspec(deprecated)
#endif

namespace synapse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arming = 0;
      this->joy = 0;
      this->fuel = 0;
      this->mode = 0;
      this->safety = 0;
      this->synapse = 0;
      this->status_message = "";
      this->fuel_percentage = 0;
      this->power = 0.0f;
      this->request_seq = 0ul;
      this->request_rejected = false;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arming = 0;
      this->joy = 0;
      this->fuel = 0;
      this->mode = 0;
      this->safety = 0;
      this->synapse = 0;
      this->status_message = "";
      this->fuel_percentage = 0;
      this->power = 0.0f;
      this->request_seq = 0ul;
      this->request_rejected = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _arming_type =
    uint8_t;
  _arming_type arming;
  using _joy_type =
    uint8_t;
  _joy_type joy;
  using _fuel_type =
    uint8_t;
  _fuel_type fuel;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _safety_type =
    uint8_t;
  _safety_type safety;
  using _synapse_type =
    uint8_t;
  _synapse_type synapse;
  using _status_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_message_type status_message;
  using _fuel_percentage_type =
    uint8_t;
  _fuel_percentage_type fuel_percentage;
  using _power_type =
    float;
  _power_type power;
  using _request_seq_type =
    uint32_t;
  _request_seq_type request_seq;
  using _request_rejected_type =
    bool;
  _request_rejected_type request_rejected;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__arming(
    const uint8_t & _arg)
  {
    this->arming = _arg;
    return *this;
  }
  Type & set__joy(
    const uint8_t & _arg)
  {
    this->joy = _arg;
    return *this;
  }
  Type & set__fuel(
    const uint8_t & _arg)
  {
    this->fuel = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__safety(
    const uint8_t & _arg)
  {
    this->safety = _arg;
    return *this;
  }
  Type & set__synapse(
    const uint8_t & _arg)
  {
    this->synapse = _arg;
    return *this;
  }
  Type & set__status_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_message = _arg;
    return *this;
  }
  Type & set__fuel_percentage(
    const uint8_t & _arg)
  {
    this->fuel_percentage = _arg;
    return *this;
  }
  Type & set__power(
    const float & _arg)
  {
    this->power = _arg;
    return *this;
  }
  Type & set__request_seq(
    const uint32_t & _arg)
  {
    this->request_seq = _arg;
    return *this;
  }
  Type & set__request_rejected(
    const bool & _arg)
  {
    this->request_rejected = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ARMING_UNKNOWN =
    0u;
  static constexpr uint8_t ARMING_DISARMED =
    1u;
  static constexpr uint8_t ARMING_ARMED =
    2u;
  static constexpr uint8_t FUEL_UNKNOWN =
    0u;
  static constexpr uint8_t FUEL_CRITICAL =
    1u;
  static constexpr uint8_t FUEL_LOW =
    2u;
  static constexpr uint8_t FUEL_NOMINAL =
    3u;
  static constexpr uint8_t FUEL_OVER_CAPACITY =
    4u;
  static constexpr uint8_t JOY_UNKNOWN =
    0u;
  static constexpr uint8_t JOY_DISABLED =
    1u;
  static constexpr uint8_t JOY_LOSS =
    2u;
  static constexpr uint8_t JOY_NOMINAL =
    3u;
  static constexpr uint8_t MODE_UNKNOWN =
    0u;
  static constexpr uint8_t MODE_MANUAL =
    1u;
  static constexpr uint8_t MODE_AUTO =
    2u;
  static constexpr uint8_t MODE_CMD_VEL =
    3u;
  static constexpr uint8_t MODE_CALIBRATION =
    4u;
  static constexpr uint8_t SAFETY_UNKNOWN =
    0u;
  static constexpr uint8_t SAFETY_SAFE =
    1u;
  static constexpr uint8_t SAFETY_UNSAFE =
    2u;
  static constexpr uint8_t SYNAPSE_UNKNOWN =
    0u;
  static constexpr uint8_t SYNAPSE_DISABLED =
    1u;
  static constexpr uint8_t SYNAPSE_LOSS =
    2u;
  static constexpr uint8_t SYNAPSE_NOMINAL =
    3u;

  // pointer types
  using RawPtr =
    synapse_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const synapse_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<synapse_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<synapse_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<synapse_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<synapse_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__synapse_msgs__msg__Status
    std::shared_ptr<synapse_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__synapse_msgs__msg__Status
    std::shared_ptr<synapse_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->arming != other.arming) {
      return false;
    }
    if (this->joy != other.joy) {
      return false;
    }
    if (this->fuel != other.fuel) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->safety != other.safety) {
      return false;
    }
    if (this->synapse != other.synapse) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    if (this->fuel_percentage != other.fuel_percentage) {
      return false;
    }
    if (this->power != other.power) {
      return false;
    }
    if (this->request_seq != other.request_seq) {
      return false;
    }
    if (this->request_rejected != other.request_rejected) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  synapse_msgs::msg::Status_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::ARMING_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::ARMING_DISARMED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::ARMING_ARMED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FUEL_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FUEL_CRITICAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FUEL_LOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FUEL_NOMINAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::FUEL_OVER_CAPACITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::JOY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::JOY_DISABLED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::JOY_LOSS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::JOY_NOMINAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MODE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MODE_MANUAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MODE_AUTO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MODE_CMD_VEL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::MODE_CALIBRATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SAFETY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SAFETY_SAFE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SAFETY_UNSAFE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SYNAPSE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SYNAPSE_DISABLED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SYNAPSE_LOSS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::SYNAPSE_NOMINAL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
