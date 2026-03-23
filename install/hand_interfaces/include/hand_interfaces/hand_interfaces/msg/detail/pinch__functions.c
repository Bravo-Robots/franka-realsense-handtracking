// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hand_interfaces:msg/Pinch.idl
// generated code does not contain a copyright notice
#include "hand_interfaces/msg/detail/pinch__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `wrist`
// Member `thumb`
// Member `index`
// Member `middle`
// Member `ring`
// Member `pinky`
#include "hand_interfaces/msg/detail/finger_data__functions.h"

bool
hand_interfaces__msg__Pinch__init(hand_interfaces__msg__Pinch * msg)
{
  if (!msg) {
    return false;
  }
  // wrist
  if (!hand_interfaces__msg__FingerData__init(&msg->wrist)) {
    hand_interfaces__msg__Pinch__fini(msg);
    return false;
  }
  // thumb
  if (!hand_interfaces__msg__FingerData__init(&msg->thumb)) {
    hand_interfaces__msg__Pinch__fini(msg);
    return false;
  }
  // index
  if (!hand_interfaces__msg__FingerData__init(&msg->index)) {
    hand_interfaces__msg__Pinch__fini(msg);
    return false;
  }
  // middle
  if (!hand_interfaces__msg__FingerData__init(&msg->middle)) {
    hand_interfaces__msg__Pinch__fini(msg);
    return false;
  }
  // ring
  if (!hand_interfaces__msg__FingerData__init(&msg->ring)) {
    hand_interfaces__msg__Pinch__fini(msg);
    return false;
  }
  // pinky
  if (!hand_interfaces__msg__FingerData__init(&msg->pinky)) {
    hand_interfaces__msg__Pinch__fini(msg);
    return false;
  }
  return true;
}

void
hand_interfaces__msg__Pinch__fini(hand_interfaces__msg__Pinch * msg)
{
  if (!msg) {
    return;
  }
  // wrist
  hand_interfaces__msg__FingerData__fini(&msg->wrist);
  // thumb
  hand_interfaces__msg__FingerData__fini(&msg->thumb);
  // index
  hand_interfaces__msg__FingerData__fini(&msg->index);
  // middle
  hand_interfaces__msg__FingerData__fini(&msg->middle);
  // ring
  hand_interfaces__msg__FingerData__fini(&msg->ring);
  // pinky
  hand_interfaces__msg__FingerData__fini(&msg->pinky);
}

bool
hand_interfaces__msg__Pinch__are_equal(const hand_interfaces__msg__Pinch * lhs, const hand_interfaces__msg__Pinch * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // wrist
  if (!hand_interfaces__msg__FingerData__are_equal(
      &(lhs->wrist), &(rhs->wrist)))
  {
    return false;
  }
  // thumb
  if (!hand_interfaces__msg__FingerData__are_equal(
      &(lhs->thumb), &(rhs->thumb)))
  {
    return false;
  }
  // index
  if (!hand_interfaces__msg__FingerData__are_equal(
      &(lhs->index), &(rhs->index)))
  {
    return false;
  }
  // middle
  if (!hand_interfaces__msg__FingerData__are_equal(
      &(lhs->middle), &(rhs->middle)))
  {
    return false;
  }
  // ring
  if (!hand_interfaces__msg__FingerData__are_equal(
      &(lhs->ring), &(rhs->ring)))
  {
    return false;
  }
  // pinky
  if (!hand_interfaces__msg__FingerData__are_equal(
      &(lhs->pinky), &(rhs->pinky)))
  {
    return false;
  }
  return true;
}

bool
hand_interfaces__msg__Pinch__copy(
  const hand_interfaces__msg__Pinch * input,
  hand_interfaces__msg__Pinch * output)
{
  if (!input || !output) {
    return false;
  }
  // wrist
  if (!hand_interfaces__msg__FingerData__copy(
      &(input->wrist), &(output->wrist)))
  {
    return false;
  }
  // thumb
  if (!hand_interfaces__msg__FingerData__copy(
      &(input->thumb), &(output->thumb)))
  {
    return false;
  }
  // index
  if (!hand_interfaces__msg__FingerData__copy(
      &(input->index), &(output->index)))
  {
    return false;
  }
  // middle
  if (!hand_interfaces__msg__FingerData__copy(
      &(input->middle), &(output->middle)))
  {
    return false;
  }
  // ring
  if (!hand_interfaces__msg__FingerData__copy(
      &(input->ring), &(output->ring)))
  {
    return false;
  }
  // pinky
  if (!hand_interfaces__msg__FingerData__copy(
      &(input->pinky), &(output->pinky)))
  {
    return false;
  }
  return true;
}

hand_interfaces__msg__Pinch *
hand_interfaces__msg__Pinch__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__msg__Pinch * msg = (hand_interfaces__msg__Pinch *)allocator.allocate(sizeof(hand_interfaces__msg__Pinch), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hand_interfaces__msg__Pinch));
  bool success = hand_interfaces__msg__Pinch__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hand_interfaces__msg__Pinch__destroy(hand_interfaces__msg__Pinch * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hand_interfaces__msg__Pinch__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hand_interfaces__msg__Pinch__Sequence__init(hand_interfaces__msg__Pinch__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__msg__Pinch * data = NULL;

  if (size) {
    data = (hand_interfaces__msg__Pinch *)allocator.zero_allocate(size, sizeof(hand_interfaces__msg__Pinch), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hand_interfaces__msg__Pinch__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hand_interfaces__msg__Pinch__fini(&data[i - 1]);
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
hand_interfaces__msg__Pinch__Sequence__fini(hand_interfaces__msg__Pinch__Sequence * array)
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
      hand_interfaces__msg__Pinch__fini(&array->data[i]);
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

hand_interfaces__msg__Pinch__Sequence *
hand_interfaces__msg__Pinch__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__msg__Pinch__Sequence * array = (hand_interfaces__msg__Pinch__Sequence *)allocator.allocate(sizeof(hand_interfaces__msg__Pinch__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hand_interfaces__msg__Pinch__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hand_interfaces__msg__Pinch__Sequence__destroy(hand_interfaces__msg__Pinch__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hand_interfaces__msg__Pinch__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hand_interfaces__msg__Pinch__Sequence__are_equal(const hand_interfaces__msg__Pinch__Sequence * lhs, const hand_interfaces__msg__Pinch__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hand_interfaces__msg__Pinch__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hand_interfaces__msg__Pinch__Sequence__copy(
  const hand_interfaces__msg__Pinch__Sequence * input,
  hand_interfaces__msg__Pinch__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hand_interfaces__msg__Pinch);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hand_interfaces__msg__Pinch * data =
      (hand_interfaces__msg__Pinch *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hand_interfaces__msg__Pinch__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hand_interfaces__msg__Pinch__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hand_interfaces__msg__Pinch__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
