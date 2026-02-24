// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pendulum2_msgs:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_STATE__TRAITS_HPP_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pendulum2_msgs/msg/detail/joint_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pendulum2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointState & msg,
  std::ostream & out)
{
  out << "{";
  // member: pole_angle
  {
    out << "pole_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pole_angle, out);
    out << ", ";
  }

  // member: pole_velocity
  {
    out << "pole_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.pole_velocity, out);
    out << ", ";
  }

  // member: cart_position
  {
    out << "cart_position: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_position, out);
    out << ", ";
  }

  // member: cart_velocity
  {
    out << "cart_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_velocity, out);
    out << ", ";
  }

  // member: cart_force
  {
    out << "cart_force: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pole_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pole_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pole_angle, out);
    out << "\n";
  }

  // member: pole_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pole_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.pole_velocity, out);
    out << "\n";
  }

  // member: cart_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_position: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_position, out);
    out << "\n";
  }

  // member: cart_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_velocity, out);
    out << "\n";
  }

  // member: cart_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cart_force: ";
    rosidl_generator_traits::value_to_yaml(msg.cart_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointState & msg, bool use_flow_style = false)
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
  const pendulum2_msgs::msg::JointState & msg,
  std::ostream & out, size_t indentation = 0)
{
  pendulum2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pendulum2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const pendulum2_msgs::msg::JointState & msg)
{
  return pendulum2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pendulum2_msgs::msg::JointState>()
{
  return "pendulum2_msgs::msg::JointState";
}

template<>
inline const char * name<pendulum2_msgs::msg::JointState>()
{
  return "pendulum2_msgs/msg/JointState";
}

template<>
struct has_fixed_size<pendulum2_msgs::msg::JointState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pendulum2_msgs::msg::JointState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pendulum2_msgs::msg::JointState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_STATE__TRAITS_HPP_
