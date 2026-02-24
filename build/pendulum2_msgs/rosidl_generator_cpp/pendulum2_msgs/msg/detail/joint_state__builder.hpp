// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pendulum2_msgs:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pendulum2_msgs/msg/detail/joint_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pendulum2_msgs
{

namespace msg
{

namespace builder
{

class Init_JointState_cart_force
{
public:
  explicit Init_JointState_cart_force(::pendulum2_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  ::pendulum2_msgs::msg::JointState cart_force(::pendulum2_msgs::msg::JointState::_cart_force_type arg)
  {
    msg_.cart_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointState msg_;
};

class Init_JointState_cart_velocity
{
public:
  explicit Init_JointState_cart_velocity(::pendulum2_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_cart_force cart_velocity(::pendulum2_msgs::msg::JointState::_cart_velocity_type arg)
  {
    msg_.cart_velocity = std::move(arg);
    return Init_JointState_cart_force(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointState msg_;
};

class Init_JointState_cart_position
{
public:
  explicit Init_JointState_cart_position(::pendulum2_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_cart_velocity cart_position(::pendulum2_msgs::msg::JointState::_cart_position_type arg)
  {
    msg_.cart_position = std::move(arg);
    return Init_JointState_cart_velocity(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointState msg_;
};

class Init_JointState_pole_velocity
{
public:
  explicit Init_JointState_pole_velocity(::pendulum2_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_cart_position pole_velocity(::pendulum2_msgs::msg::JointState::_pole_velocity_type arg)
  {
    msg_.pole_velocity = std::move(arg);
    return Init_JointState_cart_position(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointState msg_;
};

class Init_JointState_pole_angle
{
public:
  Init_JointState_pole_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointState_pole_velocity pole_angle(::pendulum2_msgs::msg::JointState::_pole_angle_type arg)
  {
    msg_.pole_angle = std::move(arg);
    return Init_JointState_pole_velocity(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pendulum2_msgs::msg::JointState>()
{
  return pendulum2_msgs::msg::builder::Init_JointState_pole_angle();
}

}  // namespace pendulum2_msgs

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
