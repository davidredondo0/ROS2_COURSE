// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from as2_msgs:msg/PoseWithIDArray.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__MSG__DETAIL__POSE_WITH_ID_ARRAY__STRUCT_HPP_
#define AS2_MSGS__MSG__DETAIL__POSE_WITH_ID_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'poses'
#include "as2_msgs/msg/detail/pose_with_id__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__as2_msgs__msg__PoseWithIDArray __attribute__((deprecated))
#else
# define DEPRECATED__as2_msgs__msg__PoseWithIDArray __declspec(deprecated)
#endif

namespace as2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseWithIDArray_
{
  using Type = PoseWithIDArray_<ContainerAllocator>;

  explicit PoseWithIDArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PoseWithIDArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _poses_type =
    std::vector<as2_msgs::msg::PoseWithID_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<as2_msgs::msg::PoseWithID_<ContainerAllocator>>>;
  _poses_type poses;

  // setters for named parameter idiom
  Type & set__poses(
    const std::vector<as2_msgs::msg::PoseWithID_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<as2_msgs::msg::PoseWithID_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    as2_msgs::msg::PoseWithIDArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const as2_msgs::msg::PoseWithIDArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      as2_msgs::msg::PoseWithIDArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      as2_msgs::msg::PoseWithIDArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__as2_msgs__msg__PoseWithIDArray
    std::shared_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__as2_msgs__msg__PoseWithIDArray
    std::shared_ptr<as2_msgs::msg::PoseWithIDArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseWithIDArray_ & other) const
  {
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseWithIDArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseWithIDArray_

// alias to use template instance with default allocator
using PoseWithIDArray =
  as2_msgs::msg::PoseWithIDArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace as2_msgs

#endif  // AS2_MSGS__MSG__DETAIL__POSE_WITH_ID_ARRAY__STRUCT_HPP_
