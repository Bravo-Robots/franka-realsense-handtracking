// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hand_interfaces:msg/Pinch.idl
// generated code does not contain a copyright notice
#include "hand_interfaces/msg/detail/pinch__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hand_interfaces/msg/detail/pinch__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace hand_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const hand_interfaces::msg::FingerData &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  hand_interfaces::msg::FingerData &);
size_t get_serialized_size(
  const hand_interfaces::msg::FingerData &,
  size_t current_alignment);
size_t
max_serialized_size_FingerData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace hand_interfaces

// functions for hand_interfaces::msg::FingerData already declared above

// functions for hand_interfaces::msg::FingerData already declared above

// functions for hand_interfaces::msg::FingerData already declared above

// functions for hand_interfaces::msg::FingerData already declared above

// functions for hand_interfaces::msg::FingerData already declared above


namespace hand_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hand_interfaces
cdr_serialize(
  const hand_interfaces::msg::Pinch & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: wrist
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.wrist,
    cdr);
  // Member: thumb
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.thumb,
    cdr);
  // Member: index
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.index,
    cdr);
  // Member: middle
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.middle,
    cdr);
  // Member: ring
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.ring,
    cdr);
  // Member: pinky
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pinky,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hand_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hand_interfaces::msg::Pinch & ros_message)
{
  // Member: wrist
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.wrist);

  // Member: thumb
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.thumb);

  // Member: index
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.index);

  // Member: middle
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.middle);

  // Member: ring
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.ring);

  // Member: pinky
  hand_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pinky);

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hand_interfaces
get_serialized_size(
  const hand_interfaces::msg::Pinch & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: wrist

  current_alignment +=
    hand_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.wrist, current_alignment);
  // Member: thumb

  current_alignment +=
    hand_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.thumb, current_alignment);
  // Member: index

  current_alignment +=
    hand_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.index, current_alignment);
  // Member: middle

  current_alignment +=
    hand_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.middle, current_alignment);
  // Member: ring

  current_alignment +=
    hand_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.ring, current_alignment);
  // Member: pinky

  current_alignment +=
    hand_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pinky, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hand_interfaces
max_serialized_size_Pinch(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: wrist
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        hand_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_FingerData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: thumb
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        hand_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_FingerData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: index
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        hand_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_FingerData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: middle
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        hand_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_FingerData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: ring
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        hand_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_FingerData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: pinky
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        hand_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_FingerData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hand_interfaces::msg::Pinch;
    is_plain =
      (
      offsetof(DataType, pinky) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Pinch__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hand_interfaces::msg::Pinch *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Pinch__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hand_interfaces::msg::Pinch *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Pinch__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hand_interfaces::msg::Pinch *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Pinch__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Pinch(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Pinch__callbacks = {
  "hand_interfaces::msg",
  "Pinch",
  _Pinch__cdr_serialize,
  _Pinch__cdr_deserialize,
  _Pinch__get_serialized_size,
  _Pinch__max_serialized_size
};

static rosidl_message_type_support_t _Pinch__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Pinch__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hand_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hand_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<hand_interfaces::msg::Pinch>()
{
  return &hand_interfaces::msg::typesupport_fastrtps_cpp::_Pinch__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hand_interfaces, msg, Pinch)() {
  return &hand_interfaces::msg::typesupport_fastrtps_cpp::_Pinch__handle;
}

#ifdef __cplusplus
}
#endif
