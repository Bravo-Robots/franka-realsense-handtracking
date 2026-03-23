// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hand_interfaces:srv/HandPose.idl
// generated code does not contain a copyright notice

#ifndef HAND_INTERFACES__SRV__DETAIL__HAND_POSE__STRUCT_H_
#define HAND_INTERFACES__SRV__DETAIL__HAND_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/HandPose in the package hand_interfaces.
typedef struct hand_interfaces__srv__HandPose_Request
{
  geometry_msgs__msg__PoseStamped pose;
} hand_interfaces__srv__HandPose_Request;

// Struct for a sequence of hand_interfaces__srv__HandPose_Request.
typedef struct hand_interfaces__srv__HandPose_Request__Sequence
{
  hand_interfaces__srv__HandPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hand_interfaces__srv__HandPose_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/HandPose in the package hand_interfaces.
typedef struct hand_interfaces__srv__HandPose_Response
{
  uint8_t structure_needs_at_least_one_member;
} hand_interfaces__srv__HandPose_Response;

// Struct for a sequence of hand_interfaces__srv__HandPose_Response.
typedef struct hand_interfaces__srv__HandPose_Response__Sequence
{
  hand_interfaces__srv__HandPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hand_interfaces__srv__HandPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HAND_INTERFACES__SRV__DETAIL__HAND_POSE__STRUCT_H_
