// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from pendulum2_msgs:msg/PendulumTeleop.idl
// generated code does not contain a copyright notice
#include "pendulum2_msgs/msg/detail/pendulum_teleop__rosidl_typesupport_fastrtps_cpp.hpp"
#include "pendulum2_msgs/msg/detail/pendulum_teleop__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace pendulum2_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pendulum2_msgs
cdr_serialize(
  const pendulum2_msgs::msg::PendulumTeleop & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pole_angle
  cdr << ros_message.pole_angle;
  // Member: pole_velocity
  cdr << ros_message.pole_velocity;
  // Member: cart_position
  cdr << ros_message.cart_position;
  // Member: cart_velocity
  cdr << ros_message.cart_velocity;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pendulum2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  pendulum2_msgs::msg::PendulumTeleop & ros_message)
{
  // Member: pole_angle
  cdr >> ros_message.pole_angle;

  // Member: pole_velocity
  cdr >> ros_message.pole_velocity;

  // Member: cart_position
  cdr >> ros_message.cart_position;

  // Member: cart_velocity
  cdr >> ros_message.cart_velocity;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pendulum2_msgs
get_serialized_size(
  const pendulum2_msgs::msg::PendulumTeleop & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pole_angle
  {
    size_t item_size = sizeof(ros_message.pole_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pole_velocity
  {
    size_t item_size = sizeof(ros_message.pole_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cart_position
  {
    size_t item_size = sizeof(ros_message.cart_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cart_velocity
  {
    size_t item_size = sizeof(ros_message.cart_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pendulum2_msgs
max_serialized_size_PendulumTeleop(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: pole_angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: pole_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: cart_position
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: cart_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pendulum2_msgs::msg::PendulumTeleop;
    is_plain =
      (
      offsetof(DataType, cart_velocity) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PendulumTeleop__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const pendulum2_msgs::msg::PendulumTeleop *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PendulumTeleop__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<pendulum2_msgs::msg::PendulumTeleop *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PendulumTeleop__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const pendulum2_msgs::msg::PendulumTeleop *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PendulumTeleop__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PendulumTeleop(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PendulumTeleop__callbacks = {
  "pendulum2_msgs::msg",
  "PendulumTeleop",
  _PendulumTeleop__cdr_serialize,
  _PendulumTeleop__cdr_deserialize,
  _PendulumTeleop__get_serialized_size,
  _PendulumTeleop__max_serialized_size
};

static rosidl_message_type_support_t _PendulumTeleop__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PendulumTeleop__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace pendulum2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_pendulum2_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<pendulum2_msgs::msg::PendulumTeleop>()
{
  return &pendulum2_msgs::msg::typesupport_fastrtps_cpp::_PendulumTeleop__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pendulum2_msgs, msg, PendulumTeleop)() {
  return &pendulum2_msgs::msg::typesupport_fastrtps_cpp::_PendulumTeleop__handle;
}

#ifdef __cplusplus
}
#endif
