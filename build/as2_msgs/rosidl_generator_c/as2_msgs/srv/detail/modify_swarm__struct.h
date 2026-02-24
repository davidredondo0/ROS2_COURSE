// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from as2_msgs:srv/ModifySwarm.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__STRUCT_H_
#define AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'virtual_centroid'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'swarm_formation'
#include "as2_msgs/msg/detail/pose_with_id__struct.h"

/// Struct defined in srv/ModifySwarm in the package as2_msgs.
typedef struct as2_msgs__srv__ModifySwarm_Request
{
  /// Detach the drone reference from the swarm
  bool detach_drone;
  /// Add new drone reference to the swarm
  bool new_drone;
  /// New reference to follow
  bool new_virtual_centroid_ref;
  /// Offset of the virtual centroid to the following frame
  geometry_msgs__msg__PoseStamped virtual_centroid;
  /// Topics to modify the flocking
  as2_msgs__msg__PoseWithID__Sequence swarm_formation;
} as2_msgs__srv__ModifySwarm_Request;

// Struct for a sequence of as2_msgs__srv__ModifySwarm_Request.
typedef struct as2_msgs__srv__ModifySwarm_Request__Sequence
{
  as2_msgs__srv__ModifySwarm_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__srv__ModifySwarm_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ModifySwarm in the package as2_msgs.
typedef struct as2_msgs__srv__ModifySwarm_Response
{
  /// whether the SwarmBehavior has been set or not
  bool success;
} as2_msgs__srv__ModifySwarm_Response;

// Struct for a sequence of as2_msgs__srv__ModifySwarm_Response.
typedef struct as2_msgs__srv__ModifySwarm_Response__Sequence
{
  as2_msgs__srv__ModifySwarm_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__srv__ModifySwarm_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__STRUCT_H_
