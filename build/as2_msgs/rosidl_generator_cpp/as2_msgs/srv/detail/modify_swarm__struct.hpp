// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from as2_msgs:srv/ModifySwarm.idl
// generated code does not contain a copyright notice

#ifndef AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__STRUCT_HPP_
#define AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'virtual_centroid'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'swarm_formation'
#include "as2_msgs/msg/detail/pose_with_id__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__as2_msgs__srv__ModifySwarm_Request __attribute__((deprecated))
#else
# define DEPRECATED__as2_msgs__srv__ModifySwarm_Request __declspec(deprecated)
#endif

namespace as2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModifySwarm_Request_
{
  using Type = ModifySwarm_Request_<ContainerAllocator>;

  explicit ModifySwarm_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : virtual_centroid(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detach_drone = false;
      this->new_drone = false;
      this->new_virtual_centroid_ref = false;
    }
  }

  explicit ModifySwarm_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : virtual_centroid(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detach_drone = false;
      this->new_drone = false;
      this->new_virtual_centroid_ref = false;
    }
  }

  // field types and members
  using _detach_drone_type =
    bool;
  _detach_drone_type detach_drone;
  using _new_drone_type =
    bool;
  _new_drone_type new_drone;
  using _new_virtual_centroid_ref_type =
    bool;
  _new_virtual_centroid_ref_type new_virtual_centroid_ref;
  using _virtual_centroid_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _virtual_centroid_type virtual_centroid;
  using _swarm_formation_type =
    std::vector<as2_msgs::msg::PoseWithID_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<as2_msgs::msg::PoseWithID_<ContainerAllocator>>>;
  _swarm_formation_type swarm_formation;

  // setters for named parameter idiom
  Type & set__detach_drone(
    const bool & _arg)
  {
    this->detach_drone = _arg;
    return *this;
  }
  Type & set__new_drone(
    const bool & _arg)
  {
    this->new_drone = _arg;
    return *this;
  }
  Type & set__new_virtual_centroid_ref(
    const bool & _arg)
  {
    this->new_virtual_centroid_ref = _arg;
    return *this;
  }
  Type & set__virtual_centroid(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->virtual_centroid = _arg;
    return *this;
  }
  Type & set__swarm_formation(
    const std::vector<as2_msgs::msg::PoseWithID_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<as2_msgs::msg::PoseWithID_<ContainerAllocator>>> & _arg)
  {
    this->swarm_formation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__as2_msgs__srv__ModifySwarm_Request
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__as2_msgs__srv__ModifySwarm_Request
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModifySwarm_Request_ & other) const
  {
    if (this->detach_drone != other.detach_drone) {
      return false;
    }
    if (this->new_drone != other.new_drone) {
      return false;
    }
    if (this->new_virtual_centroid_ref != other.new_virtual_centroid_ref) {
      return false;
    }
    if (this->virtual_centroid != other.virtual_centroid) {
      return false;
    }
    if (this->swarm_formation != other.swarm_formation) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModifySwarm_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModifySwarm_Request_

// alias to use template instance with default allocator
using ModifySwarm_Request =
  as2_msgs::srv::ModifySwarm_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace as2_msgs


#ifndef _WIN32
# define DEPRECATED__as2_msgs__srv__ModifySwarm_Response __attribute__((deprecated))
#else
# define DEPRECATED__as2_msgs__srv__ModifySwarm_Response __declspec(deprecated)
#endif

namespace as2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModifySwarm_Response_
{
  using Type = ModifySwarm_Response_<ContainerAllocator>;

  explicit ModifySwarm_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ModifySwarm_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__as2_msgs__srv__ModifySwarm_Response
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__as2_msgs__srv__ModifySwarm_Response
    std::shared_ptr<as2_msgs::srv::ModifySwarm_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModifySwarm_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModifySwarm_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModifySwarm_Response_

// alias to use template instance with default allocator
using ModifySwarm_Response =
  as2_msgs::srv::ModifySwarm_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace as2_msgs

namespace as2_msgs
{

namespace srv
{

struct ModifySwarm
{
  using Request = as2_msgs::srv::ModifySwarm_Request;
  using Response = as2_msgs::srv::ModifySwarm_Response;
};

}  // namespace srv

}  // namespace as2_msgs

#endif  // AS2_MSGS__SRV__DETAIL__MODIFY_SWARM__STRUCT_HPP_
