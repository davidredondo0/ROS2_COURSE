// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from as2_msgs:srv/ModifySwarm.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__TRAITS_HPP_
#define AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "as2_msgs/srv/detail/modify_swarm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'virtual_centroid'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'swarm_formation'
#include "as2_msgs/msg/detail/pose_with_id__traits.hpp"

namespace as2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ModifySwarm_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: detach_drone
  {
    out << "detach_drone: ";
    rosidl_generator_traits::value_to_yaml(msg.detach_drone, out);
    out << ", ";
  }

  // member: new_drone
  {
    out << "new_drone: ";
    rosidl_generator_traits::value_to_yaml(msg.new_drone, out);
    out << ", ";
  }

  // member: new_virtual_centroid_ref
  {
    out << "new_virtual_centroid_ref: ";
    rosidl_generator_traits::value_to_yaml(msg.new_virtual_centroid_ref, out);
    out << ", ";
  }

  // member: virtual_centroid
  {
    out << "virtual_centroid: ";
    to_flow_style_yaml(msg.virtual_centroid, out);
    out << ", ";
  }

  // member: swarm_formation
  {
    if (msg.swarm_formation.size() == 0) {
      out << "swarm_formation: []";
    } else {
      out << "swarm_formation: [";
      size_t pending_items = msg.swarm_formation.size();
      for (auto item : msg.swarm_formation) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ModifySwarm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: detach_drone
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detach_drone: ";
    rosidl_generator_traits::value_to_yaml(msg.detach_drone, out);
    out << "\n";
  }

  // member: new_drone
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_drone: ";
    rosidl_generator_traits::value_to_yaml(msg.new_drone, out);
    out << "\n";
  }

  // member: new_virtual_centroid_ref
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_virtual_centroid_ref: ";
    rosidl_generator_traits::value_to_yaml(msg.new_virtual_centroid_ref, out);
    out << "\n";
  }

  // member: virtual_centroid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "virtual_centroid:\n";
    to_block_style_yaml(msg.virtual_centroid, out, indentation + 2);
  }

  // member: swarm_formation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.swarm_formation.size() == 0) {
      out << "swarm_formation: []\n";
    } else {
      out << "swarm_formation:\n";
      for (auto item : msg.swarm_formation) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ModifySwarm_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace as2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use as2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const as2_msgs::srv::ModifySwarm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  as2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use as2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const as2_msgs::srv::ModifySwarm_Request & msg)
{
  return as2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<as2_msgs::srv::ModifySwarm_Request>()
{
  return "as2_msgs::srv::ModifySwarm_Request";
}

template<>
inline const char * name<as2_msgs::srv::ModifySwarm_Request>()
{
  return "as2_msgs/srv/ModifySwarm_Request";
}

template<>
struct has_fixed_size<as2_msgs::srv::ModifySwarm_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<as2_msgs::srv::ModifySwarm_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<as2_msgs::srv::ModifySwarm_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace as2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ModifySwarm_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ModifySwarm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ModifySwarm_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace as2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use as2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const as2_msgs::srv::ModifySwarm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  as2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use as2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const as2_msgs::srv::ModifySwarm_Response & msg)
{
  return as2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<as2_msgs::srv::ModifySwarm_Response>()
{
  return "as2_msgs::srv::ModifySwarm_Response";
}

template<>
inline const char * name<as2_msgs::srv::ModifySwarm_Response>()
{
  return "as2_msgs/srv/ModifySwarm_Response";
}

template<>
struct has_fixed_size<as2_msgs::srv::ModifySwarm_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<as2_msgs::srv::ModifySwarm_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<as2_msgs::srv::ModifySwarm_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<as2_msgs::srv::ModifySwarm>()
{
  return "as2_msgs::srv::ModifySwarm";
}

template<>
inline const char * name<as2_msgs::srv::ModifySwarm>()
{
  return "as2_msgs/srv/ModifySwarm";
}

template<>
struct has_fixed_size<as2_msgs::srv::ModifySwarm>
  : std::integral_constant<
    bool,
    has_fixed_size<as2_msgs::srv::ModifySwarm_Request>::value &&
    has_fixed_size<as2_msgs::srv::ModifySwarm_Response>::value
  >
{
};

template<>
struct has_bounded_size<as2_msgs::srv::ModifySwarm>
  : std::integral_constant<
    bool,
    has_bounded_size<as2_msgs::srv::ModifySwarm_Request>::value &&
    has_bounded_size<as2_msgs::srv::ModifySwarm_Response>::value
  >
{
};

template<>
struct is_service<as2_msgs::srv::ModifySwarm>
  : std::true_type
{
};

template<>
struct is_service_request<as2_msgs::srv::ModifySwarm_Request>
  : std::true_type
{
};

template<>
struct is_service_response<as2_msgs::srv::ModifySwarm_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__TRAITS_HPP_
