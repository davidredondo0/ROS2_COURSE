// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pendulum2_msgs:msg/JointCommandStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pendulum2_msgs/msg/detail/joint_command_stamped__rosidl_typesupport_introspection_c.h"
#include "pendulum2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pendulum2_msgs/msg/detail/joint_command_stamped__functions.h"
#include "pendulum2_msgs/msg/detail/joint_command_stamped__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `cmd`
#include "pendulum2_msgs/msg/joint_command.h"
// Member `cmd`
#include "pendulum2_msgs/msg/detail/joint_command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pendulum2_msgs__msg__JointCommandStamped__init(message_memory);
}

void pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_fini_function(void * message_memory)
{
  pendulum2_msgs__msg__JointCommandStamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs__msg__JointCommandStamped, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cmd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pendulum2_msgs__msg__JointCommandStamped, cmd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_members = {
  "pendulum2_msgs__msg",  // message namespace
  "JointCommandStamped",  // message name
  2,  // number of fields
  sizeof(pendulum2_msgs__msg__JointCommandStamped),
  pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_member_array,  // message members
  pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_type_support_handle = {
  0,
  &pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pendulum2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pendulum2_msgs, msg, JointCommandStamped)() {
  pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pendulum2_msgs, msg, JointCommand)();
  if (!pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_type_support_handle.typesupport_identifier) {
    pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pendulum2_msgs__msg__JointCommandStamped__rosidl_typesupport_introspection_c__JointCommandStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
