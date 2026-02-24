// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pendulum2_msgs:msg/PendulumTeleop.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__PENDULUM_TELEOP__STRUCT_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__PENDULUM_TELEOP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pendulum2_msgs__msg__PendulumTeleop __attribute__((deprecated))
#else
# define DEPRECATED__pendulum2_msgs__msg__PendulumTeleop __declspec(deprecated)
#endif

namespace pendulum2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PendulumTeleop_
{
  using Type = PendulumTeleop_<ContainerAllocator>;

  explicit PendulumTeleop_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pole_angle = 0.0;
      this->pole_velocity = 0.0;
      this->cart_position = 0.0;
      this->cart_velocity = 0.0;
    }
  }

  explicit PendulumTeleop_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pole_angle = 0.0;
      this->pole_velocity = 0.0;
      this->cart_position = 0.0;
      this->cart_velocity = 0.0;
    }
  }

  // field types and members
  using _pole_angle_type =
    double;
  _pole_angle_type pole_angle;
  using _pole_velocity_type =
    double;
  _pole_velocity_type pole_velocity;
  using _cart_position_type =
    double;
  _cart_position_type cart_position;
  using _cart_velocity_type =
    double;
  _cart_velocity_type cart_velocity;

  // setters for named parameter idiom
  Type & set__pole_angle(
    const double & _arg)
  {
    this->pole_angle = _arg;
    return *this;
  }
  Type & set__pole_velocity(
    const double & _arg)
  {
    this->pole_velocity = _arg;
    return *this;
  }
  Type & set__cart_position(
    const double & _arg)
  {
    this->cart_position = _arg;
    return *this;
  }
  Type & set__cart_velocity(
    const double & _arg)
  {
    this->cart_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator> *;
  using ConstRawPtr =
    const pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pendulum2_msgs__msg__PendulumTeleop
    std::shared_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pendulum2_msgs__msg__PendulumTeleop
    std::shared_ptr<pendulum2_msgs::msg::PendulumTeleop_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PendulumTeleop_ & other) const
  {
    if (this->pole_angle != other.pole_angle) {
      return false;
    }
    if (this->pole_velocity != other.pole_velocity) {
      return false;
    }
    if (this->cart_position != other.cart_position) {
      return false;
    }
    if (this->cart_velocity != other.cart_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const PendulumTeleop_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PendulumTeleop_

// alias to use template instance with default allocator
using PendulumTeleop =
  pendulum2_msgs::msg::PendulumTeleop_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pendulum2_msgs

#endif  // PENDULUM2_MSGS__MSG__DETAIL__PENDULUM_TELEOP__STRUCT_HPP_
