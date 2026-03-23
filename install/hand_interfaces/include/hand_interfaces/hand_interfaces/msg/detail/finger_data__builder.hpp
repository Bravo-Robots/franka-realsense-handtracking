// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hand_interfaces:msg/FingerData.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__BUILDER_HPP_
#define HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hand_interfaces/msg/detail/finger_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hand_interfaces
{

namespace msg
{

namespace builder
{

class Init_FingerData_y
{
public:
  explicit Init_FingerData_y(::hand_interfaces::msg::FingerData & msg)
  : msg_(msg)
  {}
  ::hand_interfaces::msg::FingerData y(::hand_interfaces::msg::FingerData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hand_interfaces::msg::FingerData msg_;
};

class Init_FingerData_x
{
public:
  Init_FingerData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FingerData_y x(::hand_interfaces::msg::FingerData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_FingerData_y(msg_);
  }

private:
  ::hand_interfaces::msg::FingerData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hand_interfaces::msg::FingerData>()
{
  return hand_interfaces::msg::builder::Init_FingerData_x();
}

}  // namespace hand_interfaces

#endif  // HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__BUILDER_HPP_
