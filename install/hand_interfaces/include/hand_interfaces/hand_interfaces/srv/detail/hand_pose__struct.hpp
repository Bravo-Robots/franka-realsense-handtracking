// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hand_interfaces:srv/HandPose.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__SRV__DETAIL__HAND_POSE__STRUCT_HPP_
#define HAND_INTERFACES__SRV__DETAIL__HAND_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hand_interfaces__srv__HandPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__hand_interfaces__srv__HandPose_Request __declspec(deprecated)
#endif

namespace hand_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HandPose_Request_
{
  using Type = HandPose_Request_<ContainerAllocator>;

  explicit HandPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    (void)_init;
  }

  explicit HandPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hand_interfaces::srv::HandPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hand_interfaces::srv::HandPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hand_interfaces::srv::HandPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hand_interfaces::srv::HandPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hand_interfaces__srv__HandPose_Request
    std::shared_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hand_interfaces__srv__HandPose_Request
    std::shared_ptr<hand_interfaces::srv::HandPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandPose_Request_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandPose_Request_

// alias to use template instance with default allocator
using HandPose_Request =
  hand_interfaces::srv::HandPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hand_interfaces


#ifndef _WIN32
# define DEPRECATED__hand_interfaces__srv__HandPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__hand_interfaces__srv__HandPose_Response __declspec(deprecated)
#endif

namespace hand_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HandPose_Response_
{
  using Type = HandPose_Response_<ContainerAllocator>;

  explicit HandPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit HandPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    hand_interfaces::srv::HandPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hand_interfaces::srv::HandPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hand_interfaces::srv::HandPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hand_interfaces::srv::HandPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hand_interfaces__srv__HandPose_Response
    std::shared_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hand_interfaces__srv__HandPose_Response
    std::shared_ptr<hand_interfaces::srv::HandPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandPose_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandPose_Response_

// alias to use template instance with default allocator
using HandPose_Response =
  hand_interfaces::srv::HandPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hand_interfaces

namespace hand_interfaces
{

namespace srv
{

struct HandPose
{
  using Request = hand_interfaces::srv::HandPose_Request;
  using Response = hand_interfaces::srv::HandPose_Response;
};

}  // namespace srv

}  // namespace hand_interfaces

#endif  // HAND_INTERFACES__SRV__DETAIL__HAND_POSE__STRUCT_HPP_
