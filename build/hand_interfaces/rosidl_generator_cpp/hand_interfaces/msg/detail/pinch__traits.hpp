// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hand_interfaces:msg/Pinch.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__MSG__DETAIL__PINCH__TRAITS_HPP_
#define HAND_INTERFACES__MSG__DETAIL__PINCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hand_interfaces/msg/detail/pinch__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'wrist'
// Member 'thumb'
// Member 'index'
// Member 'middle'
// Member 'ring'
// Member 'pinky'
#include "hand_interfaces/msg/detail/finger_data__traits.hpp"

namespace hand_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pinch & msg,
  std::ostream & out)
{
  out << "{";
  // member: wrist
  {
    out << "wrist: ";
    to_flow_style_yaml(msg.wrist, out);
    out << ", ";
  }

  // member: thumb
  {
    out << "thumb: ";
    to_flow_style_yaml(msg.thumb, out);
    out << ", ";
  }

  // member: index
  {
    out << "index: ";
    to_flow_style_yaml(msg.index, out);
    out << ", ";
  }

  // member: middle
  {
    out << "middle: ";
    to_flow_style_yaml(msg.middle, out);
    out << ", ";
  }

  // member: ring
  {
    out << "ring: ";
    to_flow_style_yaml(msg.ring, out);
    out << ", ";
  }

  // member: pinky
  {
    out << "pinky: ";
    to_flow_style_yaml(msg.pinky, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pinch & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: wrist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrist:\n";
    to_block_style_yaml(msg.wrist, out, indentation + 2);
  }

  // member: thumb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thumb:\n";
    to_block_style_yaml(msg.thumb, out, indentation + 2);
  }

  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index:\n";
    to_block_style_yaml(msg.index, out, indentation + 2);
  }

  // member: middle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "middle:\n";
    to_block_style_yaml(msg.middle, out, indentation + 2);
  }

  // member: ring
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ring:\n";
    to_block_style_yaml(msg.ring, out, indentation + 2);
  }

  // member: pinky
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pinky:\n";
    to_block_style_yaml(msg.pinky, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pinch & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hand_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hand_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hand_interfaces::msg::Pinch & msg,
  std::ostream & out, size_t indentation = 0)
{
  hand_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hand_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hand_interfaces::msg::Pinch & msg)
{
  return hand_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hand_interfaces::msg::Pinch>()
{
  return "hand_interfaces::msg::Pinch";
}

template<>
inline const char * name<hand_interfaces::msg::Pinch>()
{
  return "hand_interfaces/msg/Pinch";
}

template<>
struct has_fixed_size<hand_interfaces::msg::Pinch>
  : std::integral_constant<bool, has_fixed_size<hand_interfaces::msg::FingerData>::value> {};

template<>
struct has_bounded_size<hand_interfaces::msg::Pinch>
  : std::integral_constant<bool, has_bounded_size<hand_interfaces::msg::FingerData>::value> {};

template<>
struct is_message<hand_interfaces::msg::Pinch>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HAND_INTERFACES__MSG__DETAIL__PINCH__TRAITS_HPP_
