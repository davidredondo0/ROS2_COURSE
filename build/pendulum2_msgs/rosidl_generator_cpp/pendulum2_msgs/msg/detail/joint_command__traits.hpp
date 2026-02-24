// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pendulum2_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND__TRAITS_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pendulum2_msgs/msg/detail/joint_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pendulum2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointCommand & msg, bool use_flow_style = false)
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
  const pendulum2_msgs::msg::JointCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  pendulum2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pendulum2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const pendulum2_msgs::msg::JointCommand & msg)
{
  return pendulum2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pendulum2_msgs::msg::JointCommand>()
{
  return "pendulum2_msgs::msg::JointCommand";
}

template<>
inline const char * name<pendulum2_msgs::msg::JointCommand>()
{
  return "pendulum2_msgs/msg/JointCommand";
}

template<>
struct has_fixed_size<pendulum2_msgs::msg::JointCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pendulum2_msgs::msg::JointCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pendulum2_msgs::msg::JointCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND__TRAITS_HPP_
