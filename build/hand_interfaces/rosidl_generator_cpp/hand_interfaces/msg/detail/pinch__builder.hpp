// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hand_interfaces:msg/Pinch.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__MSG__DETAIL__PINCH__BUILDER_HPP_
#define HAND_INTERFACES__MSG__DETAIL__PINCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hand_interfaces/msg/detail/pinch__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hand_interfaces
{

namespace msg
{

namespace builder
{

class Init_Pinch_pinky
{
public:
  explicit Init_Pinch_pinky(::hand_interfaces::msg::Pinch & msg)
  : msg_(msg)
  {}
  ::hand_interfaces::msg::Pinch pinky(::hand_interfaces::msg::Pinch::_pinky_type arg)
  {
    msg_.pinky = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hand_interfaces::msg::Pinch msg_;
};

class Init_Pinch_ring
{
public:
  explicit Init_Pinch_ring(::hand_interfaces::msg::Pinch & msg)
  : msg_(msg)
  {}
  Init_Pinch_pinky ring(::hand_interfaces::msg::Pinch::_ring_type arg)
  {
    msg_.ring = std::move(arg);
    return Init_Pinch_pinky(msg_);
  }

private:
  ::hand_interfaces::msg::Pinch msg_;
};

class Init_Pinch_middle
{
public:
  explicit Init_Pinch_middle(::hand_interfaces::msg::Pinch & msg)
  : msg_(msg)
  {}
  Init_Pinch_ring middle(::hand_interfaces::msg::Pinch::_middle_type arg)
  {
    msg_.middle = std::move(arg);
    return Init_Pinch_ring(msg_);
  }

private:
  ::hand_interfaces::msg::Pinch msg_;
};

class Init_Pinch_index
{
public:
  explicit Init_Pinch_index(::hand_interfaces::msg::Pinch & msg)
  : msg_(msg)
  {}
  Init_Pinch_middle index(::hand_interfaces::msg::Pinch::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_Pinch_middle(msg_);
  }

private:
  ::hand_interfaces::msg::Pinch msg_;
};

class Init_Pinch_thumb
{
public:
  explicit Init_Pinch_thumb(::hand_interfaces::msg::Pinch & msg)
  : msg_(msg)
  {}
  Init_Pinch_index thumb(::hand_interfaces::msg::Pinch::_thumb_type arg)
  {
    msg_.thumb = std::move(arg);
    return Init_Pinch_index(msg_);
  }

private:
  ::hand_interfaces::msg::Pinch msg_;
};

class Init_Pinch_wrist
{
public:
  Init_Pinch_wrist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pinch_thumb wrist(::hand_interfaces::msg::Pinch::_wrist_type arg)
  {
    msg_.wrist = std::move(arg);
    return Init_Pinch_thumb(msg_);
  }

private:
  ::hand_interfaces::msg::Pinch msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hand_interfaces::msg::Pinch>()
{
  return hand_interfaces::msg::builder::Init_Pinch_wrist();
}

}  // namespace hand_interfaces

#endif  // HAND_INTERFACES__MSG__DETAIL__PINCH__BUILDER_HPP_
