// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from as2_msgs:msg/UInt16MultiArrayStamped.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__STRUCT_H_
#define AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'layout'
#include "std_msgs/msg/detail/multi_array_layout__struct.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/UInt16MultiArrayStamped in the package as2_msgs.
/**
  * Please look at the std_msgs/MultiArrayLayout message definition for
  * documentation on all multiarrays.
  * This message is a multiarray of uint16 values with a timestamp, based on
  * the std_msgs/MultiArrayLayout message.
 */
typedef struct as2_msgs__msg__UInt16MultiArrayStamped
{
  /// Message timestamp
  builtin_interfaces__msg__Time stamp;
  /// Specification of data layout
  std_msgs__msg__MultiArrayLayout layout;
  /// Array of data
  rosidl_runtime_c__uint16__Sequence data;
} as2_msgs__msg__UInt16MultiArrayStamped;

// Struct for a sequence of as2_msgs__msg__UInt16MultiArrayStamped.
typedef struct as2_msgs__msg__UInt16MultiArrayStamped__Sequence
{
  as2_msgs__msg__UInt16MultiArrayStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__msg__UInt16MultiArrayStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__STRUCT_H_
