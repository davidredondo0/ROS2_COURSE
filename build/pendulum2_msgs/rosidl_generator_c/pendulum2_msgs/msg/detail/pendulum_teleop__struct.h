// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pendulum2_msgs:msg/PendulumTeleop.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__PENDULUM_TELEOP__STRUCT_H_
#define PENDULUM2_MSGS__MSG__DETAIL__PENDULUM_TELEOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PendulumTeleop in the package pendulum2_msgs.
/**
  * This represents the desired state of the pendulum for teleoperation
 */
typedef struct pendulum2_msgs__msg__PendulumTeleop
{
  double pole_angle;
  double pole_velocity;
  double cart_position;
  double cart_velocity;
} pendulum2_msgs__msg__PendulumTeleop;

// Struct for a sequence of pendulum2_msgs__msg__PendulumTeleop.
typedef struct pendulum2_msgs__msg__PendulumTeleop__Sequence
{
  pendulum2_msgs__msg__PendulumTeleop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pendulum2_msgs__msg__PendulumTeleop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PENDULUM2_MSGS__MSG__DETAIL__PENDULUM_TELEOP__STRUCT_H_
