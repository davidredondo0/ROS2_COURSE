// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pendulum2_msgs:msg/JointCommandStamped.idl
// generated code does not contain a copyright notice

#ifndef PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__FUNCTIONS_H_
#define PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pendulum2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pendulum2_msgs/msg/detail/joint_command_stamped__struct.h"

/// Initialize msg/JointCommandStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pendulum2_msgs__msg__JointCommandStamped
 * )) before or use
 * pendulum2_msgs__msg__JointCommandStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
bool
pendulum2_msgs__msg__JointCommandStamped__init(pendulum2_msgs__msg__JointCommandStamped * msg);

/// Finalize msg/JointCommandStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
void
pendulum2_msgs__msg__JointCommandStamped__fini(pendulum2_msgs__msg__JointCommandStamped * msg);

/// Create msg/JointCommandStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pendulum2_msgs__msg__JointCommandStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
pendulum2_msgs__msg__JointCommandStamped *
pendulum2_msgs__msg__JointCommandStamped__create();

/// Destroy msg/JointCommandStamped message.
/**
 * It calls
 * pendulum2_msgs__msg__JointCommandStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
void
pendulum2_msgs__msg__JointCommandStamped__destroy(pendulum2_msgs__msg__JointCommandStamped * msg);

/// Check for msg/JointCommandStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
bool
pendulum2_msgs__msg__JointCommandStamped__are_equal(const pendulum2_msgs__msg__JointCommandStamped * lhs, const pendulum2_msgs__msg__JointCommandStamped * rhs);

/// Copy a msg/JointCommandStamped message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
bool
pendulum2_msgs__msg__JointCommandStamped__copy(
  const pendulum2_msgs__msg__JointCommandStamped * input,
  pendulum2_msgs__msg__JointCommandStamped * output);

/// Initialize array of msg/JointCommandStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * pendulum2_msgs__msg__JointCommandStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
bool
pendulum2_msgs__msg__JointCommandStamped__Sequence__init(pendulum2_msgs__msg__JointCommandStamped__Sequence * array, size_t size);

/// Finalize array of msg/JointCommandStamped messages.
/**
 * It calls
 * pendulum2_msgs__msg__JointCommandStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
void
pendulum2_msgs__msg__JointCommandStamped__Sequence__fini(pendulum2_msgs__msg__JointCommandStamped__Sequence * array);

/// Create array of msg/JointCommandStamped messages.
/**
 * It allocates the memory for the array and calls
 * pendulum2_msgs__msg__JointCommandStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
pendulum2_msgs__msg__JointCommandStamped__Sequence *
pendulum2_msgs__msg__JointCommandStamped__Sequence__create(size_t size);

/// Destroy array of msg/JointCommandStamped messages.
/**
 * It calls
 * pendulum2_msgs__msg__JointCommandStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
void
pendulum2_msgs__msg__JointCommandStamped__Sequence__destroy(pendulum2_msgs__msg__JointCommandStamped__Sequence * array);

/// Check for msg/JointCommandStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
bool
pendulum2_msgs__msg__JointCommandStamped__Sequence__are_equal(const pendulum2_msgs__msg__JointCommandStamped__Sequence * lhs, const pendulum2_msgs__msg__JointCommandStamped__Sequence * rhs);

/// Copy an array of msg/JointCommandStamped messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pendulum2_msgs
bool
pendulum2_msgs__msg__JointCommandStamped__Sequence__copy(
  const pendulum2_msgs__msg__JointCommandStamped__Sequence * input,
  pendulum2_msgs__msg__JointCommandStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PENDULUM2_MSGS__MSG__DETAIL__JOINT_COMMAND_STAMPED__FUNCTIONS_H_
