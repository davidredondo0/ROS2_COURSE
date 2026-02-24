// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from as2_msgs:msg/UInt16MultiArrayStamped.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__STRUCT_HPP_
#define AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'layout'
#include "std_msgs/msg/detail/multi_array_layout__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__as2_msgs__msg__UInt16MultiArrayStamped __attribute__((deprecated))
#else
# define DEPRECATED__as2_msgs__msg__UInt16MultiArrayStamped __declspec(deprecated)
#endif

namespace as2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt16MultiArrayStamped_
{
  using Type = UInt16MultiArrayStamped_<ContainerAllocator>;

  explicit UInt16MultiArrayStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    layout(_init)
  {
    (void)_init;
  }

  explicit UInt16MultiArrayStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__as2_msgs__msg__UInt16MultiArrayStamped
    std::shared_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__as2_msgs__msg__UInt16MultiArrayStamped
    std::shared_ptr<as2_msgs::msg::UInt16MultiArrayStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt16MultiArrayStamped_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt16MultiArrayStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt16MultiArrayStamped_

// alias to use template instance with default allocator
using UInt16MultiArrayStamped =
  as2_msgs::msg::UInt16MultiArrayStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace as2_msgs

#endif  // AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__STRUCT_HPP_
