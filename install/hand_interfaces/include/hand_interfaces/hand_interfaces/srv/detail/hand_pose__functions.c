// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hand_interfaces:srv/HandPose.idl
// generated code does not contain a copyright notice
#include "hand_interfaces/srv/detail/hand_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
hand_interfaces__srv__HandPose_Request__init(hand_interfaces__srv__HandPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    hand_interfaces__srv__HandPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
hand_interfaces__srv__HandPose_Request__fini(hand_interfaces__srv__HandPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
}

bool
hand_interfaces__srv__HandPose_Request__are_equal(const hand_interfaces__srv__HandPose_Request * lhs, const hand_interfaces__srv__HandPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
hand_interfaces__srv__HandPose_Request__copy(
  const hand_interfaces__srv__HandPose_Request * input,
  hand_interfaces__srv__HandPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

hand_interfaces__srv__HandPose_Request *
hand_interfaces__srv__HandPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__srv__HandPose_Request * msg = (hand_interfaces__srv__HandPose_Request *)allocator.allocate(sizeof(hand_interfaces__srv__HandPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hand_interfaces__srv__HandPose_Request));
  bool success = hand_interfaces__srv__HandPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hand_interfaces__srv__HandPose_Request__destroy(hand_interfaces__srv__HandPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hand_interfaces__srv__HandPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hand_interfaces__srv__HandPose_Request__Sequence__init(hand_interfaces__srv__HandPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__srv__HandPose_Request * data = NULL;

  if (size) {
    data = (hand_interfaces__srv__HandPose_Request *)allocator.zero_allocate(size, sizeof(hand_interfaces__srv__HandPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hand_interfaces__srv__HandPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hand_interfaces__srv__HandPose_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hand_interfaces__srv__HandPose_Request__Sequence__fini(hand_interfaces__srv__HandPose_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hand_interfaces__srv__HandPose_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hand_interfaces__srv__HandPose_Request__Sequence *
hand_interfaces__srv__HandPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__srv__HandPose_Request__Sequence * array = (hand_interfaces__srv__HandPose_Request__Sequence *)allocator.allocate(sizeof(hand_interfaces__srv__HandPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hand_interfaces__srv__HandPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hand_interfaces__srv__HandPose_Request__Sequence__destroy(hand_interfaces__srv__HandPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hand_interfaces__srv__HandPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hand_interfaces__srv__HandPose_Request__Sequence__are_equal(const hand_interfaces__srv__HandPose_Request__Sequence * lhs, const hand_interfaces__srv__HandPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hand_interfaces__srv__HandPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hand_interfaces__srv__HandPose_Request__Sequence__copy(
  const hand_interfaces__srv__HandPose_Request__Sequence * input,
  hand_interfaces__srv__HandPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hand_interfaces__srv__HandPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hand_interfaces__srv__HandPose_Request * data =
      (hand_interfaces__srv__HandPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hand_interfaces__srv__HandPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hand_interfaces__srv__HandPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hand_interfaces__srv__HandPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
hand_interfaces__srv__HandPose_Response__init(hand_interfaces__srv__HandPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
hand_interfaces__srv__HandPose_Response__fini(hand_interfaces__srv__HandPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
hand_interfaces__srv__HandPose_Response__are_equal(const hand_interfaces__srv__HandPose_Response * lhs, const hand_interfaces__srv__HandPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
hand_interfaces__srv__HandPose_Response__copy(
  const hand_interfaces__srv__HandPose_Response * input,
  hand_interfaces__srv__HandPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

hand_interfaces__srv__HandPose_Response *
hand_interfaces__srv__HandPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__srv__HandPose_Response * msg = (hand_interfaces__srv__HandPose_Response *)allocator.allocate(sizeof(hand_interfaces__srv__HandPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hand_interfaces__srv__HandPose_Response));
  bool success = hand_interfaces__srv__HandPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hand_interfaces__srv__HandPose_Response__destroy(hand_interfaces__srv__HandPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hand_interfaces__srv__HandPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hand_interfaces__srv__HandPose_Response__Sequence__init(hand_interfaces__srv__HandPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__srv__HandPose_Response * data = NULL;

  if (size) {
    data = (hand_interfaces__srv__HandPose_Response *)allocator.zero_allocate(size, sizeof(hand_interfaces__srv__HandPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hand_interfaces__srv__HandPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hand_interfaces__srv__HandPose_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hand_interfaces__srv__HandPose_Response__Sequence__fini(hand_interfaces__srv__HandPose_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hand_interfaces__srv__HandPose_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hand_interfaces__srv__HandPose_Response__Sequence *
hand_interfaces__srv__HandPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__srv__HandPose_Response__Sequence * array = (hand_interfaces__srv__HandPose_Response__Sequence *)allocator.allocate(sizeof(hand_interfaces__srv__HandPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hand_interfaces__srv__HandPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hand_interfaces__srv__HandPose_Response__Sequence__destroy(hand_interfaces__srv__HandPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hand_interfaces__srv__HandPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hand_interfaces__srv__HandPose_Response__Sequence__are_equal(const hand_interfaces__srv__HandPose_Response__Sequence * lhs, const hand_interfaces__srv__HandPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hand_interfaces__srv__HandPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hand_interfaces__srv__HandPose_Response__Sequence__copy(
  const hand_interfaces__srv__HandPose_Response__Sequence * input,
  hand_interfaces__srv__HandPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hand_interfaces__srv__HandPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hand_interfaces__srv__HandPose_Response * data =
      (hand_interfaces__srv__HandPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hand_interfaces__srv__HandPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hand_interfaces__srv__HandPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hand_interfaces__srv__HandPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
