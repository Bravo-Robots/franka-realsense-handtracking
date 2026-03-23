// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hand_interfaces:srv/HandPose.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__SRV__DETAIL__HAND_POSE__BUILDER_HPP_
#define HAND_INTERFACES__SRV__DETAIL__HAND_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hand_interfaces/srv/detail/hand_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hand_interfaces
{

namespace srv
{

namespace builder
{

class Init_HandPose_Request_pose
{
public:
  Init_HandPose_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hand_interfaces::srv::HandPose_Request pose(::hand_interfaces::srv::HandPose_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hand_interfaces::srv::HandPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hand_interfaces::srv::HandPose_Request>()
{
  return hand_interfaces::srv::builder::Init_HandPose_Request_pose();
}

}  // namespace hand_interfaces


namespace hand_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hand_interfaces::srv::HandPose_Response>()
{
  return ::hand_interfaces::srv::HandPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hand_interfaces

#endif  // HAND_INTERFACES__SRV__DETAIL__HAND_POSE__BUILDER_HPP_
