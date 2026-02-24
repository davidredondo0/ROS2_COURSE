// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pendulum2_msgs:msg/JointCommandStamped.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__STRUCT_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__STRUCT_HPP_

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
// Member 'cmd'
#include "pendulum2_msgs/msg/detail/joint_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pendulum2_msgs__msg__JointCommandStamped __attribute__((deprecated))
#else
# define DEPRECATED__pendulum2_msgs__msg__JointCommandStamped __declspec(deprecated)
#endif

namespace pendulum2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointCommandStamped_
{
  using Type = JointCommandStamped_<ContainerAllocator>;

  explicit JointCommandStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    cmd(_init)
  {
    (void)_init;
  }

  explicit JointCommandStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    cmd(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cmd_type =
    pendulum2_msgs::msg::JointCommand_<ContainerAllocator>;
  _cmd_type cmd;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cmd(
    const pendulum2_msgs::msg::JointCommand_<ContainerAllocator> & _arg)
  {
    this->cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pendulum2_msgs__msg__JointCommandStamped
    std::shared_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pendulum2_msgs__msg__JointCommandStamped
    std::shared_ptr<pendulum2_msgs::msg::JointCommandStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointCommandStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cmd != other.cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointCommandStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointCommandStamped_

// alias to use template instance with default allocator
using JointCommandStamped =
  pendulum2_msgs::msg::JointCommandStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pendulum2_msgs

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__STRUCT_HPP_
