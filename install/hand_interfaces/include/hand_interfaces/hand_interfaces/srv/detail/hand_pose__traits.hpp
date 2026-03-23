// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hand_interfaces:srv/HandPose.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__SRV__DETAIL__HAND_POSE__TRAITS_HPP_
#define HAND_INTERFACES__SRV__DETAIL__HAND_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hand_interfaces/srv/detail/hand_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace hand_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const HandPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HandPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HandPose_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hand_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hand_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hand_interfaces::srv::HandPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  hand_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hand_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const hand_interfaces::srv::HandPose_Request & msg)
{
  return hand_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hand_interfaces::srv::HandPose_Request>()
{
  return "hand_interfaces::srv::HandPose_Request";
}

template<>
inline const char * name<hand_interfaces::srv::HandPose_Request>()
{
  return "hand_interfaces/srv/HandPose_Request";
}

template<>
struct has_fixed_size<hand_interfaces::srv::HandPose_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<hand_interfaces::srv::HandPose_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<hand_interfaces::srv::HandPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace hand_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const HandPose_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HandPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HandPose_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hand_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hand_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hand_interfaces::srv::HandPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  hand_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hand_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const hand_interfaces::srv::HandPose_Response & msg)
{
  return hand_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hand_interfaces::srv::HandPose_Response>()
{
  return "hand_interfaces::srv::HandPose_Response";
}

template<>
inline const char * name<hand_interfaces::srv::HandPose_Response>()
{
  return "hand_interfaces/srv/HandPose_Response";
}

template<>
struct has_fixed_size<hand_interfaces::srv::HandPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hand_interfaces::srv::HandPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hand_interfaces::srv::HandPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hand_interfaces::srv::HandPose>()
{
  return "hand_interfaces::srv::HandPose";
}

template<>
inline const char * name<hand_interfaces::srv::HandPose>()
{
  return "hand_interfaces/srv/HandPose";
}

template<>
struct has_fixed_size<hand_interfaces::srv::HandPose>
  : std::integral_constant<
    bool,
    has_fixed_size<hand_interfaces::srv::HandPose_Request>::value &&
    has_fixed_size<hand_interfaces::srv::HandPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<hand_interfaces::srv::HandPose>
  : std::integral_constant<
    bool,
    has_bounded_size<hand_interfaces::srv::HandPose_Request>::value &&
    has_bounded_size<hand_interfaces::srv::HandPose_Response>::value
  >
{
};

template<>
struct is_service<hand_interfaces::srv::HandPose>
  : std::true_type
{
};

template<>
struct is_service_request<hand_interfaces::srv::HandPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hand_interfaces::srv::HandPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HAND_INTERFACES__SRV__DETAIL__HAND_POSE__TRAITS_HPP_
