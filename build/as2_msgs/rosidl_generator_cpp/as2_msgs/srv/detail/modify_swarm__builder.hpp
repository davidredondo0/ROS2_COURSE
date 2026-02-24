// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from as2_msgs:srv/ModifySwarm.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__BUILDER_HPP_
#define AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "as2_msgs/srv/detail/modify_swarm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace as2_msgs
{

namespace srv
{

namespace builder
{

class Init_ModifySwarm_Request_swarm_formation
{
public:
  explicit Init_ModifySwarm_Request_swarm_formation(::as2_msgs::srv::ModifySwarm_Request & msg)
  : msg_(msg)
  {}
  ::as2_msgs::srv::ModifySwarm_Request swarm_formation(::as2_msgs::srv::ModifySwarm_Request::_swarm_formation_type arg)
  {
    msg_.swarm_formation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::srv::ModifySwarm_Request msg_;
};

class Init_ModifySwarm_Request_virtual_centroid
{
public:
  explicit Init_ModifySwarm_Request_virtual_centroid(::as2_msgs::srv::ModifySwarm_Request & msg)
  : msg_(msg)
  {}
  Init_ModifySwarm_Request_swarm_formation virtual_centroid(::as2_msgs::srv::ModifySwarm_Request::_virtual_centroid_type arg)
  {
    msg_.virtual_centroid = std::move(arg);
    return Init_ModifySwarm_Request_swarm_formation(msg_);
  }

private:
  ::as2_msgs::srv::ModifySwarm_Request msg_;
};

class Init_ModifySwarm_Request_new_virtual_centroid_ref
{
public:
  explicit Init_ModifySwarm_Request_new_virtual_centroid_ref(::as2_msgs::srv::ModifySwarm_Request & msg)
  : msg_(msg)
  {}
  Init_ModifySwarm_Request_virtual_centroid new_virtual_centroid_ref(::as2_msgs::srv::ModifySwarm_Request::_new_virtual_centroid_ref_type arg)
  {
    msg_.new_virtual_centroid_ref = std::move(arg);
    return Init_ModifySwarm_Request_virtual_centroid(msg_);
  }

private:
  ::as2_msgs::srv::ModifySwarm_Request msg_;
};

class Init_ModifySwarm_Request_new_drone
{
public:
  explicit Init_ModifySwarm_Request_new_drone(::as2_msgs::srv::ModifySwarm_Request & msg)
  : msg_(msg)
  {}
  Init_ModifySwarm_Request_new_virtual_centroid_ref new_drone(::as2_msgs::srv::ModifySwarm_Request::_new_drone_type arg)
  {
    msg_.new_drone = std::move(arg);
    return Init_ModifySwarm_Request_new_virtual_centroid_ref(msg_);
  }

private:
  ::as2_msgs::srv::ModifySwarm_Request msg_;
};

class Init_ModifySwarm_Request_detach_drone
{
public:
  Init_ModifySwarm_Request_detach_drone()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModifySwarm_Request_new_drone detach_drone(::as2_msgs::srv::ModifySwarm_Request::_detach_drone_type arg)
  {
    msg_.detach_drone = std::move(arg);
    return Init_ModifySwarm_Request_new_drone(msg_);
  }

private:
  ::as2_msgs::srv::ModifySwarm_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::srv::ModifySwarm_Request>()
{
  return as2_msgs::srv::builder::Init_ModifySwarm_Request_detach_drone();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace srv
{

namespace builder
{

class Init_ModifySwarm_Response_success
{
public:
  Init_ModifySwarm_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::as2_msgs::srv::ModifySwarm_Response success(::as2_msgs::srv::ModifySwarm_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::srv::ModifySwarm_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::srv::ModifySwarm_Response>()
{
  return as2_msgs::srv::builder::Init_ModifySwarm_Response_success();
}

}  // namespace as2_msgs

#endif  // AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__BUILDER_HPP_
