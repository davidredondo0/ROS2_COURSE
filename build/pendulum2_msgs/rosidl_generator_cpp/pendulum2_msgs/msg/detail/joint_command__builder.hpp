// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pendulum2_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pendulum2_msgs/msg/detail/joint_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pendulum2_msgs
{

namespace msg
{

namespace builder
{

class Init_JointCommand_force
{
public:
  Init_JointCommand_force()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pendulum2_msgs::msg::JointCommand force(::pendulum2_msgs::msg::JointCommand::_force_type arg)
  {
    msg_.force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pendulum2_msgs::msg::JointCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pendulum2_msgs::msg::JointCommand>()
{
  return pendulum2_msgs::msg::builder::Init_JointCommand_force();
}

}  // namespace pendulum2_msgs

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
