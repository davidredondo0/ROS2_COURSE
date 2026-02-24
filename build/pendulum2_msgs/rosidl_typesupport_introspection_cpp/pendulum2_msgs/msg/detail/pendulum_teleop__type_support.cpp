// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pendulum2_msgs:msg/PendulumTeleop.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pendulum2_msgs/msg/detail/pendulum_teleop__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pendulum2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PendulumTeleop_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pendulum2_msgs::msg::PendulumTeleop(_init);
}

void PendulumTeleop_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pendulum2_msgs::msg::PendulumTeleop *>(message_memory);
  typed_message->~PendulumTeleop();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PendulumTeleop_message_member_array[4] = {
  {
    "pole_angle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs::msg::PendulumTeleop, pole_angle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pole_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs::msg::PendulumTeleop, pole_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cart_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs::msg::PendulumTeleop, cart_position),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cart_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs::msg::PendulumTeleop, cart_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PendulumTeleop_message_members = {
  "pendulum2_msgs::msg",  // message namespace
  "PendulumTeleop",  // message name
  4,  // number of fields
  sizeof(pendulum2_msgs::msg::PendulumTeleop),
  PendulumTeleop_message_member_array,  // message members
  PendulumTeleop_init_function,  // function to initialize message memory (memory has to be allocated)
  PendulumTeleop_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PendulumTeleop_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PendulumTeleop_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace pendulum2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pendulum2_msgs::msg::PendulumTeleop>()
{
  return &::pendulum2_msgs::msg::rosidl_typesupport_introspection_cpp::PendulumTeleop_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pendulum2_msgs, msg, PendulumTeleop)() {
  return &::pendulum2_msgs::msg::rosidl_typesupport_introspection_cpp::PendulumTeleop_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
