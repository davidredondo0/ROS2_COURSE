// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from as2_msgs:action/SwarmFlocking.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__ACTION__DETAIL__SWARM_FLOCKING__BUILDER_HPP_
#define AS2_MSGS__ACTION__DETAIL__SWARM_FLOCKING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "as2_msgs/action/detail/swarm_flocking__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_Goal_drones_namespace
{
public:
  explicit Init_SwarmFlocking_Goal_drones_namespace(::as2_msgs::action::SwarmFlocking_Goal & msg)
  : msg_(msg)
  {}
  ::as2_msgs::action::SwarmFlocking_Goal drones_namespace(::as2_msgs::action::SwarmFlocking_Goal::_drones_namespace_type arg)
  {
    msg_.drones_namespace = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_Goal msg_;
};

class Init_SwarmFlocking_Goal_swarm_formation
{
public:
  explicit Init_SwarmFlocking_Goal_swarm_formation(::as2_msgs::action::SwarmFlocking_Goal & msg)
  : msg_(msg)
  {}
  Init_SwarmFlocking_Goal_drones_namespace swarm_formation(::as2_msgs::action::SwarmFlocking_Goal::_swarm_formation_type arg)
  {
    msg_.swarm_formation = std::move(arg);
    return Init_SwarmFlocking_Goal_drones_namespace(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_Goal msg_;
};

class Init_SwarmFlocking_Goal_virtual_centroid
{
public:
  Init_SwarmFlocking_Goal_virtual_centroid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwarmFlocking_Goal_swarm_formation virtual_centroid(::as2_msgs::action::SwarmFlocking_Goal::_virtual_centroid_type arg)
  {
    msg_.virtual_centroid = std::move(arg);
    return Init_SwarmFlocking_Goal_swarm_formation(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_Goal>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_Goal_virtual_centroid();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_Result_swarm_success
{
public:
  Init_SwarmFlocking_Result_swarm_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::as2_msgs::action::SwarmFlocking_Result swarm_success(::as2_msgs::action::SwarmFlocking_Result::_swarm_success_type arg)
  {
    msg_.swarm_success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_Result>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_Result_swarm_success();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_Feedback_swarm_pose
{
public:
  Init_SwarmFlocking_Feedback_swarm_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::as2_msgs::action::SwarmFlocking_Feedback swarm_pose(::as2_msgs::action::SwarmFlocking_Feedback::_swarm_pose_type arg)
  {
    msg_.swarm_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_Feedback>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_Feedback_swarm_pose();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_SendGoal_Request_goal
{
public:
  explicit Init_SwarmFlocking_SendGoal_Request_goal(::as2_msgs::action::SwarmFlocking_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::as2_msgs::action::SwarmFlocking_SendGoal_Request goal(::as2_msgs::action::SwarmFlocking_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_SendGoal_Request msg_;
};

class Init_SwarmFlocking_SendGoal_Request_goal_id
{
public:
  Init_SwarmFlocking_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwarmFlocking_SendGoal_Request_goal goal_id(::as2_msgs::action::SwarmFlocking_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SwarmFlocking_SendGoal_Request_goal(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_SendGoal_Request>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_SendGoal_Request_goal_id();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_SendGoal_Response_stamp
{
public:
  explicit Init_SwarmFlocking_SendGoal_Response_stamp(::as2_msgs::action::SwarmFlocking_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::as2_msgs::action::SwarmFlocking_SendGoal_Response stamp(::as2_msgs::action::SwarmFlocking_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_SendGoal_Response msg_;
};

class Init_SwarmFlocking_SendGoal_Response_accepted
{
public:
  Init_SwarmFlocking_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwarmFlocking_SendGoal_Response_stamp accepted(::as2_msgs::action::SwarmFlocking_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SwarmFlocking_SendGoal_Response_stamp(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_SendGoal_Response>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_SendGoal_Response_accepted();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_GetResult_Request_goal_id
{
public:
  Init_SwarmFlocking_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::as2_msgs::action::SwarmFlocking_GetResult_Request goal_id(::as2_msgs::action::SwarmFlocking_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_GetResult_Request>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_GetResult_Request_goal_id();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_GetResult_Response_result
{
public:
  explicit Init_SwarmFlocking_GetResult_Response_result(::as2_msgs::action::SwarmFlocking_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::as2_msgs::action::SwarmFlocking_GetResult_Response result(::as2_msgs::action::SwarmFlocking_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_GetResult_Response msg_;
};

class Init_SwarmFlocking_GetResult_Response_status
{
public:
  Init_SwarmFlocking_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwarmFlocking_GetResult_Response_result status(::as2_msgs::action::SwarmFlocking_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SwarmFlocking_GetResult_Response_result(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_GetResult_Response>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_GetResult_Response_status();
}

}  // namespace as2_msgs


namespace as2_msgs
{

namespace action
{

namespace builder
{

class Init_SwarmFlocking_FeedbackMessage_feedback
{
public:
  explicit Init_SwarmFlocking_FeedbackMessage_feedback(::as2_msgs::action::SwarmFlocking_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::as2_msgs::action::SwarmFlocking_FeedbackMessage feedback(::as2_msgs::action::SwarmFlocking_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_FeedbackMessage msg_;
};

class Init_SwarmFlocking_FeedbackMessage_goal_id
{
public:
  Init_SwarmFlocking_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwarmFlocking_FeedbackMessage_feedback goal_id(::as2_msgs::action::SwarmFlocking_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SwarmFlocking_FeedbackMessage_feedback(msg_);
  }

private:
  ::as2_msgs::action::SwarmFlocking_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::action::SwarmFlocking_FeedbackMessage>()
{
  return as2_msgs::action::builder::Init_SwarmFlocking_FeedbackMessage_goal_id();
}

}  // namespace as2_msgs

#endif  // AS2_MSGS__ACTION__DETAIL__SWARM_FLOCKING__BUILDER_HPP_
