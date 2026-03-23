// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hand_interfaces:msg/FingerData.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__STRUCT_H_
#define HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/FingerData in the package hand_interfaces.
typedef struct hand_interfaces__msg__FingerData
{
  float x;
  float y;
} hand_interfaces__msg__FingerData;

// Struct for a sequence of hand_interfaces__msg__FingerData.
typedef struct hand_interfaces__msg__FingerData__Sequence
{
  hand_interfaces__msg__FingerData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hand_interfaces__msg__FingerData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HAND_INTERFACES__MSG__DETAIL__FINGER_DATA__STRUCT_H_
