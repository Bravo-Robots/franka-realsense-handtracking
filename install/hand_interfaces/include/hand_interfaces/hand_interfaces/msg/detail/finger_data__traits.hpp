// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hand_interfaces:msg/FingerData.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__TRAITS_HPP_
#define HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hand_interfaces/msg/detail/finger_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hand_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FingerData & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FingerData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FingerData & msg, bool use_flow_style = false)
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
  const hand_interfaces::msg::FingerData & msg,
  std::ostream & out, size_t indentation = 0)
{
  hand_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hand_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hand_interfaces::msg::FingerData & msg)
{
  return hand_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hand_interfaces::msg::FingerData>()
{
  return "hand_interfaces::msg::FingerData";
}

template<>
inline const char * name<hand_interfaces::msg::FingerData>()
{
  return "hand_interfaces/msg/FingerData";
}

template<>
struct has_fixed_size<hand_interfaces::msg::FingerData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hand_interfaces::msg::FingerData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hand_interfaces::msg::FingerData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__TRAITS_HPP_
