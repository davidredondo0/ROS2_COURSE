// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pendulum2_msgs:msg/JointCommandStamped.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__BUILDER_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pendulum2_msgs/msg/detail/joint_command_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pendulum2_msgs
{

namespace msg
{

namespace builder
{

class Init_JointCommandStamped_cmd
{
public:
  explicit Init_JointCommandStamped_cmd(::pendulum2_msgs::msg::JointCommandStamped & msg)
  : msg_(msg)
  {}
  ::pendulum2_msgs::msg::JointCommandStamped cmd(::pendulum2_msgs::msg::JointCommandStamped::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointCommandStamped msg_;
};

class Init_JointCommandStamped_header
{
public:
  Init_JointCommandStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointCommandStamped_cmd header(::pendulum2_msgs::msg::JointCommandStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointCommandStamped_cmd(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointCommandStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pendulum2_msgs::msg::JointCommandStamped>()
{
  return pendulum2_msgs::msg::builder::Init_JointCommandStamped_header();
}

}  // namespace pendulum2_msgs

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__BUILDER_HPP_
