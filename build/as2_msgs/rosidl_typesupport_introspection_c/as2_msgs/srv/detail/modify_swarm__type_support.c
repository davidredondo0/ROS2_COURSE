// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from as2_msgs:srv/ModifySwarm.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "as2_msgs/srv/detail/modify_swarm__rosidl_typesupport_introspection_c.h"
#include "as2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "as2_msgs/srv/detail/modify_swarm__functions.h"
#include "as2_msgs/srv/detail/modify_swarm__struct.h"


// Include directives for member types
// Member `virtual_centroid`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `virtual_centroid`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `swarm_formation`
#include "as2_msgs/msg/pose_with_id.h"
// Member `swarm_formation`
#include "as2_msgs/msg/detail/pose_with_id__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  as2_msgs__srv__ModifySwarm_Request__init(message_memory);
}

void as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_fini_function(void * message_memory)
{
  as2_msgs__srv__ModifySwarm_Request__fini(message_memory);
}

size_t as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__size_function__ModifySwarm_Request__swarm_formation(
  const void * untyped_member)
{
  const as2_msgs__msg__PoseWithID__Sequence * member =
    (const as2_msgs__msg__PoseWithID__Sequence *)(untyped_member);
  return member->size;
}

const void * as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__get_const_function__ModifySwarm_Request__swarm_formation(
  const void * untyped_member, size_t index)
{
  const as2_msgs__msg__PoseWithID__Sequence * member =
    (const as2_msgs__msg__PoseWithID__Sequence *)(untyped_member);
  return &member->data[index];
}

void * as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__get_function__ModifySwarm_Request__swarm_formation(
  void * untyped_member, size_t index)
{
  as2_msgs__msg__PoseWithID__Sequence * member =
    (as2_msgs__msg__PoseWithID__Sequence *)(untyped_member);
  return &member->data[index];
}

void as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__fetch_function__ModifySwarm_Request__swarm_formation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const as2_msgs__msg__PoseWithID * item =
    ((const as2_msgs__msg__PoseWithID *)
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__get_const_function__ModifySwarm_Request__swarm_formation(untyped_member, index));
  as2_msgs__msg__PoseWithID * value =
    (as2_msgs__msg__PoseWithID *)(untyped_value);
  *value = *item;
}

void as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__assign_function__ModifySwarm_Request__swarm_formation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  as2_msgs__msg__PoseWithID * item =
    ((as2_msgs__msg__PoseWithID *)
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__get_function__ModifySwarm_Request__swarm_formation(untyped_member, index));
  const as2_msgs__msg__PoseWithID * value =
    (const as2_msgs__msg__PoseWithID *)(untyped_value);
  *item = *value;
}

bool as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__resize_function__ModifySwarm_Request__swarm_formation(
  void * untyped_member, size_t size)
{
  as2_msgs__msg__PoseWithID__Sequence * member =
    (as2_msgs__msg__PoseWithID__Sequence *)(untyped_member);
  as2_msgs__msg__PoseWithID__Sequence__fini(member);
  return as2_msgs__msg__PoseWithID__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_member_array[5] = {
  {
    "detach_drone",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(as2_msgs__srv__ModifySwarm_Request, detach_drone),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "new_drone",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(as2_msgs__srv__ModifySwarm_Request, new_drone),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "new_virtual_centroid_ref",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(as2_msgs__srv__ModifySwarm_Request, new_virtual_centroid_ref),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "virtual_centroid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(as2_msgs__srv__ModifySwarm_Request, virtual_centroid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "swarm_formation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(as2_msgs__srv__ModifySwarm_Request, swarm_formation),  // bytes offset in struct
    NULL,  // default value
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__size_function__ModifySwarm_Request__swarm_formation,  // size() function pointer
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__get_const_function__ModifySwarm_Request__swarm_formation,  // get_const(index) function pointer
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__get_function__ModifySwarm_Request__swarm_formation,  // get(index) function pointer
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__fetch_function__ModifySwarm_Request__swarm_formation,  // fetch(index, &value) function pointer
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__assign_function__ModifySwarm_Request__swarm_formation,  // assign(index, value) function pointer
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__resize_function__ModifySwarm_Request__swarm_formation  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_members = {
  "as2_msgs__srv",  // message namespace
  "ModifySwarm_Request",  // message name
  5,  // number of fields
  sizeof(as2_msgs__srv__ModifySwarm_Request),
  as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_member_array,  // message members
  as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_type_support_handle = {
  0,
  &as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_as2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm_Request)() {
  as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, msg, PoseWithID)();
  if (!as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_type_support_handle.typesupport_identifier) {
    as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &as2_msgs__srv__ModifySwarm_Request__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "as2_msgs/srv/detail/modify_swarm__rosidl_typesupport_introspection_c.h"
// already included above
// #include "as2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "as2_msgs/srv/detail/modify_swarm__functions.h"
// already included above
// #include "as2_msgs/srv/detail/modify_swarm__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  as2_msgs__srv__ModifySwarm_Response__init(message_memory);
}

void as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_fini_function(void * message_memory)
{
  as2_msgs__srv__ModifySwarm_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(as2_msgs__srv__ModifySwarm_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_members = {
  "as2_msgs__srv",  // message namespace
  "ModifySwarm_Response",  // message name
  1,  // number of fields
  sizeof(as2_msgs__srv__ModifySwarm_Response),
  as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_member_array,  // message members
  as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_type_support_handle = {
  0,
  &as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_as2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm_Response)() {
  if (!as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_type_support_handle.typesupport_identifier) {
    as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &as2_msgs__srv__ModifySwarm_Response__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "as2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "as2_msgs/srv/detail/modify_swarm__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_members = {
  "as2_msgs__srv",  // service namespace
  "ModifySwarm",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_Request_message_type_support_handle,
  NULL  // response message
  // as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_Response_message_type_support_handle
};

static rosidl_service_type_support_t as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_type_support_handle = {
  0,
  &as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_as2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm)() {
  if (!as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_type_support_handle.typesupport_identifier) {
    as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, as2_msgs, srv, ModifySwarm_Response)()->data;
  }

  return &as2_msgs__srv__detail__modify_swarm__rosidl_typesupport_introspection_c__ModifySwarm_service_type_support_handle;
}
