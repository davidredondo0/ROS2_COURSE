// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pendulum2_msgs:msg/PendulumTeleop.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pendulum2_msgs/msg/detail/pendulum_teleop__rosidl_typesupport_introspection_c.h"
#include "pendulum2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pendulum2_msgs/msg/detail/pendulum_teleop__functions.h"
#include "pendulum2_msgs/msg/detail/pendulum_teleop__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pendulum2_msgs__msg__PendulumTeleop__init(message_memory);
}

void pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_fini_function(void * message_memory)
{
  pendulum2_msgs__msg__PendulumTeleop__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_member_array[4] = {
  {
    "pole_angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs__msg__PendulumTeleop, pole_angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pole_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs__msg__PendulumTeleop, pole_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cart_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs__msg__PendulumTeleop, cart_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cart_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs__msg__PendulumTeleop, cart_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_members = {
  "pendulum2_msgs__msg",  // message namespace
  "PendulumTeleop",  // message name
  4,  // number of fields
  sizeof(pendulum2_msgs__msg__PendulumTeleop),
  pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_member_array,  // message members
  pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_init_function,  // function to initialize message memory (memory has to be allocated)
  pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_type_support_handle = {
  0,
  &pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pendulum2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pendulum2_msgs, msg, PendulumTeleop)() {
  if (!pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_type_support_handle.typesupport_identifier) {
    pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pendulum2_msgs__msg__PendulumTeleop__rosidl_typesupport_introspection_c__PendulumTeleop_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
