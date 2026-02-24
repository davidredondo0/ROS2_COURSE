// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from as2_msgs:action/SwarmFlocking.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__ACTION__DETAIL__SWARM_FLOCKING__STRUCT_H_
#define AS2_MSGS__ACTION__DETAIL__SWARM_FLOCKING__STRUCT_H_

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
// Member 'drones_namespace'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_Goal
{
  /// Request
  /// Offset of the virtual centroid to the following frame
  geometry_msgs__msg__PoseStamped virtual_centroid;
  /// Pose of the drones with respect to the virtual centroid
  as2_msgs__msg__PoseWithID__Sequence swarm_formation;
  /// Namespaces of the drones in the swarm
  rosidl_runtime_c__String__Sequence drones_namespace;
} as2_msgs__action__SwarmFlocking_Goal;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_Goal.
typedef struct as2_msgs__action__SwarmFlocking_Goal__Sequence
{
  as2_msgs__action__SwarmFlocking_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_Result
{
  /// False if failed to swarm_success
  bool swarm_success;
} as2_msgs__action__SwarmFlocking_Result;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_Result.
typedef struct as2_msgs__action__SwarmFlocking_Result__Sequence
{
  as2_msgs__action__SwarmFlocking_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'swarm_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_Feedback
{
  /// Current swarm pose
  geometry_msgs__msg__Pose swarm_pose;
} as2_msgs__action__SwarmFlocking_Feedback;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_Feedback.
typedef struct as2_msgs__action__SwarmFlocking_Feedback__Sequence
{
  as2_msgs__action__SwarmFlocking_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "as2_msgs/action/detail/swarm_flocking__struct.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  as2_msgs__action__SwarmFlocking_Goal goal;
} as2_msgs__action__SwarmFlocking_SendGoal_Request;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_SendGoal_Request.
typedef struct as2_msgs__action__SwarmFlocking_SendGoal_Request__Sequence
{
  as2_msgs__action__SwarmFlocking_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} as2_msgs__action__SwarmFlocking_SendGoal_Response;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_SendGoal_Response.
typedef struct as2_msgs__action__SwarmFlocking_SendGoal_Response__Sequence
{
  as2_msgs__action__SwarmFlocking_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} as2_msgs__action__SwarmFlocking_GetResult_Request;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_GetResult_Request.
typedef struct as2_msgs__action__SwarmFlocking_GetResult_Request__Sequence
{
  as2_msgs__action__SwarmFlocking_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_GetResult_Response
{
  int8_t status;
  as2_msgs__action__SwarmFlocking_Result result;
} as2_msgs__action__SwarmFlocking_GetResult_Response;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_GetResult_Response.
typedef struct as2_msgs__action__SwarmFlocking_GetResult_Response__Sequence
{
  as2_msgs__action__SwarmFlocking_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"

/// Struct defined in action/SwarmFlocking in the package as2_msgs.
typedef struct as2_msgs__action__SwarmFlocking_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  as2_msgs__action__SwarmFlocking_Feedback feedback;
} as2_msgs__action__SwarmFlocking_FeedbackMessage;

// Struct for a sequence of as2_msgs__action__SwarmFlocking_FeedbackMessage.
typedef struct as2_msgs__action__SwarmFlocking_FeedbackMessage__Sequence
{
  as2_msgs__action__SwarmFlocking_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} as2_msgs__action__SwarmFlocking_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AS2_MSGS__ACTION__DETAIL__SWARM_FLOCKING__STRUCT_H_
