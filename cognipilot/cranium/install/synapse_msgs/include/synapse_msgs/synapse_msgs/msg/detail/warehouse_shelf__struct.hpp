// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__STRUCT_HPP_
#define SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__synapse_msgs__msg__WarehouseShelf __attribute__((deprecated))
#else
# define DEPRECATED__synapse_msgs__msg__WarehouseShelf __declspec(deprecated)
#endif

namespace synapse_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WarehouseShelf_
{
  using Type = WarehouseShelf_<ContainerAllocator>;

  explicit WarehouseShelf_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->qr_decoded = "";
    }
  }

  explicit WarehouseShelf_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : qr_decoded(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->qr_decoded = "";
    }
  }

  // field types and members
  using _object_name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _object_name_type object_name;
  using _object_count_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _object_count_type object_count;
  using _qr_decoded_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _qr_decoded_type qr_decoded;

  // setters for named parameter idiom
  Type & set__object_name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->object_name = _arg;
    return *this;
  }
  Type & set__object_count(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->object_count = _arg;
    return *this;
  }
  Type & set__qr_decoded(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->qr_decoded = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    synapse_msgs::msg::WarehouseShelf_<ContainerAllocator> *;
  using ConstRawPtr =
    const synapse_msgs::msg::WarehouseShelf_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::WarehouseShelf_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      synapse_msgs::msg::WarehouseShelf_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__synapse_msgs__msg__WarehouseShelf
    std::shared_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__synapse_msgs__msg__WarehouseShelf
    std::shared_ptr<synapse_msgs::msg::WarehouseShelf_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WarehouseShelf_ & other) const
  {
    if (this->object_name != other.object_name) {
      return false;
    }
    if (this->object_count != other.object_count) {
      return false;
    }
    if (this->qr_decoded != other.qr_decoded) {
      return false;
    }
    return true;
  }
  bool operator!=(const WarehouseShelf_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WarehouseShelf_

// alias to use template instance with default allocator
using WarehouseShelf =
  synapse_msgs::msg::WarehouseShelf_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace synapse_msgs

#endif  // SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__STRUCT_HPP_
