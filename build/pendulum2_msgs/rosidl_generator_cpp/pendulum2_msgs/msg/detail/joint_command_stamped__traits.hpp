// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pendulum2_msgs:msg/JointCommandStamped.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__TRAITS_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pendulum2_msgs/msg/detail/joint_command_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'cmd'
#include "pendulum2_msgs/msg/detail/joint_command__traits.hpp"

namespace pendulum2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointCommandStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: cmd
  {
    out << "cmd: ";
    to_flow_style_yaml(msg.cmd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointCommandStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd:\n";
    to_block_style_yaml(msg.cmd, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointCommandStamped & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace pendulum2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pendulum2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pendulum2_msgs::msg::JointCommandStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  pendulum2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pendulum2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const pendulum2_msgs::msg::JointCommandStamped & msg)
{
  return pendulum2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pendulum2_msgs::msg::JointCommandStamped>()
{
  return "pendulum2_msgs::msg::JointCommandStamped";
}

template<>
inline const char * name<pendulum2_msgs::msg::JointCommandStamped>()
{
  return "pendulum2_msgs/msg/JointCommandStamped";
}

template<>
struct has_fixed_size<pendulum2_msgs::msg::JointCommandStamped>
  : std::integral_constant<bool, has_fixed_size<pendulum2_msgs::msg::JointCommand>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<pendulum2_msgs::msg::JointCommandStamped>
  : std::integral_constant<bool, has_bounded_size<pendulum2_msgs::msg::JointCommand>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<pendulum2_msgs::msg::JointCommandStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__TRAITS_HPP_
