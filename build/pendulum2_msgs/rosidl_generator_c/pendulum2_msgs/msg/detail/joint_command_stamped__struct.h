// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pendulum2_msgs:msg/JointCommandStamped.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__STRUCT_H_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'cmd'
#include "pendulum2_msgs/msg/detail/joint_command__struct.h"

/// Struct defined in msg/JointCommandStamped in the package pendulum2_msgs.
/**
  * This represents a JointCommand with reference coordinate frame and timestamp
 */
typedef struct pendulum2_msgs__msg__JointCommandStamped
{
  std_msgs__msg__Header header;
  pendulum2_msgs__msg__JointCommand cmd;
} pendulum2_msgs__msg__JointCommandStamped;

// Struct for a sequence of pendulum2_msgs__msg__JointCommandStamped.
typedef struct pendulum2_msgs__msg__JointCommandStamped__Sequence
{
  pendulum2_msgs__msg__JointCommandStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pendulum2_msgs__msg__JointCommandStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__STRUCT_H_
