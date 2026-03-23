// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hand_interfaces:msg/Pinch.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__MSG__DETAIL__PINCH__STRUCT_HPP_
#define HAND_INTERFACES__MSG__DETAIL__PINCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'wrist'
// Member 'thumb'
// Member 'index'
// Member 'middle'
// Member 'ring'
// Member 'pinky'
#include "hand_interfaces/msg/detail/finger_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hand_interfaces__msg__Pinch __attribute__((deprecated))
#else
# define DEPRECATED__hand_interfaces__msg__Pinch __declspec(deprecated)
#endif

namespace hand_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pinch_
{
  using Type = Pinch_<ContainerAllocator>;

  explicit Pinch_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : wrist(_init),
    thumb(_init),
    index(_init),
    middle(_init),
    ring(_init),
    pinky(_init)
  {
    (void)_init;
  }

  explicit Pinch_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : wrist(_alloc, _init),
    thumb(_alloc, _init),
    index(_alloc, _init),
    middle(_alloc, _init),
    ring(_alloc, _init),
    pinky(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _wrist_type =
    hand_interfaces::msg::FingerData_<ContainerAllocator>;
  _wrist_type wrist;
  using _thumb_type =
    hand_interfaces::msg::FingerData_<ContainerAllocator>;
  _thumb_type thumb;
  using _index_type =
    hand_interfaces::msg::FingerData_<ContainerAllocator>;
  _index_type index;
  using _middle_type =
    hand_interfaces::msg::FingerData_<ContainerAllocator>;
  _middle_type middle;
  using _ring_type =
    hand_interfaces::msg::FingerData_<ContainerAllocator>;
  _ring_type ring;
  using _pinky_type =
    hand_interfaces::msg::FingerData_<ContainerAllocator>;
  _pinky_type pinky;

  // setters for named parameter idiom
  Type & set__wrist(
    const hand_interfaces::msg::FingerData_<ContainerAllocator> & _arg)
  {
    this->wrist = _arg;
    return *this;
  }
  Type & set__thumb(
    const hand_interfaces::msg::FingerData_<ContainerAllocator> & _arg)
  {
    this->thumb = _arg;
    return *this;
  }
  Type & set__index(
    const hand_interfaces::msg::FingerData_<ContainerAllocator> & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__middle(
    const hand_interfaces::msg::FingerData_<ContainerAllocator> & _arg)
  {
    this->middle = _arg;
    return *this;
  }
  Type & set__ring(
    const hand_interfaces::msg::FingerData_<ContainerAllocator> & _arg)
  {
    this->ring = _arg;
    return *this;
  }
  Type & set__pinky(
    const hand_interfaces::msg::FingerData_<ContainerAllocator> & _arg)
  {
    this->pinky = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hand_interfaces::msg::Pinch_<ContainerAllocator> *;
  using ConstRawPtr =
    const hand_interfaces::msg::Pinch_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hand_interfaces::msg::Pinch_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hand_interfaces::msg::Pinch_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hand_interfaces__msg__Pinch
    std::shared_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hand_interfaces__msg__Pinch
    std::shared_ptr<hand_interfaces::msg::Pinch_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pinch_ & other) const
  {
    if (this->wrist != other.wrist) {
      return false;
    }
    if (this->thumb != other.thumb) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->middle != other.middle) {
      return false;
    }
    if (this->ring != other.ring) {
      return false;
    }
    if (this->pinky != other.pinky) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pinch_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pinch_

// alias to use template instance with default allocator
using Pinch =
  hand_interfaces::msg::Pinch_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hand_interfaces

#endif  // HAND_INTERFACES__MSG__DETAIL__PINCH__STRUCT_HPP_
