// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from as2_msgs:msg/UInt16MultiArrayStamped.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__BUILDER_HPP_
#define AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "as2_msgs/msg/detail/u_int16_multi_array_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace as2_msgs
{

namespace msg
{

namespace builder
{

class Init_UInt16MultiArrayStamped_data
{
public:
  explicit Init_UInt16MultiArrayStamped_data(::as2_msgs::msg::UInt16MultiArrayStamped & msg)
  : msg_(msg)
  {}
  ::as2_msgs::msg::UInt16MultiArrayStamped data(::as2_msgs::msg::UInt16MultiArrayStamped::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::as2_msgs::msg::UInt16MultiArrayStamped msg_;
};

class Init_UInt16MultiArrayStamped_layout
{
public:
  explicit Init_UInt16MultiArrayStamped_layout(::as2_msgs::msg::UInt16MultiArrayStamped & msg)
  : msg_(msg)
  {}
  Init_UInt16MultiArrayStamped_data layout(::as2_msgs::msg::UInt16MultiArrayStamped::_layout_type arg)
  {
    msg_.layout = std::move(arg);
    return Init_UInt16MultiArrayStamped_data(msg_);
  }

private:
  ::as2_msgs::msg::UInt16MultiArrayStamped msg_;
};

class Init_UInt16MultiArrayStamped_stamp
{
public:
  Init_UInt16MultiArrayStamped_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UInt16MultiArrayStamped_layout stamp(::as2_msgs::msg::UInt16MultiArrayStamped::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_UInt16MultiArrayStamped_layout(msg_);
  }

private:
  ::as2_msgs::msg::UInt16MultiArrayStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::as2_msgs::msg::UInt16MultiArrayStamped>()
{
  return as2_msgs::msg::builder::Init_UInt16MultiArrayStamped_stamp();
}

}  // namespace as2_msgs

#endif  // AS2_MSGS__MSG__DETAIL__U_INT16_MULTI_ARRAY_STAMPED__BUILDER_HPP_
