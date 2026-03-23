// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hand_interfaces:msg/FingerData.idl
// generated code does not contain a copyright notice
#include "hand_interfaces/msg/detail/finger_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hand_interfaces__msg__FingerData__init(hand_interfaces__msg__FingerData * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
hand_interfaces__msg__FingerData__fini(hand_interfaces__msg__FingerData * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
hand_interfaces__msg__FingerData__are_equal(const hand_interfaces__msg__FingerData * lhs, const hand_interfaces__msg__FingerData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
hand_interfaces__msg__FingerData__copy(
  const hand_interfaces__msg__FingerData * input,
  hand_interfaces__msg__FingerData * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

hand_interfaces__msg__FingerData *
hand_interfaces__msg__FingerData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__msg__FingerData * msg = (hand_interfaces__msg__FingerData *)allocator.allocate(sizeof(hand_interfaces__msg__FingerData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hand_interfaces__msg__FingerData));
  bool success = hand_interfaces__msg__FingerData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hand_interfaces__msg__FingerData__destroy(hand_interfaces__msg__FingerData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hand_interfaces__msg__FingerData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hand_interfaces__msg__FingerData__Sequence__init(hand_interfaces__msg__FingerData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__msg__FingerData * data = NULL;

  if (size) {
    data = (hand_interfaces__msg__FingerData *)allocator.zero_allocate(size, sizeof(hand_interfaces__msg__FingerData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hand_interfaces__msg__FingerData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hand_interfaces__msg__FingerData__fini(&data[i - 1]);
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
hand_interfaces__msg__FingerData__Sequence__fini(hand_interfaces__msg__FingerData__Sequence * array)
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
      hand_interfaces__msg__FingerData__fini(&array->data[i]);
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

hand_interfaces__msg__FingerData__Sequence *
hand_interfaces__msg__FingerData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hand_interfaces__msg__FingerData__Sequence * array = (hand_interfaces__msg__FingerData__Sequence *)allocator.allocate(sizeof(hand_interfaces__msg__FingerData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hand_interfaces__msg__FingerData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hand_interfaces__msg__FingerData__Sequence__destroy(hand_interfaces__msg__FingerData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hand_interfaces__msg__FingerData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hand_interfaces__msg__FingerData__Sequence__are_equal(const hand_interfaces__msg__FingerData__Sequence * lhs, const hand_interfaces__msg__FingerData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hand_interfaces__msg__FingerData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hand_interfaces__msg__FingerData__Sequence__copy(
  const hand_interfaces__msg__FingerData__Sequence * input,
  hand_interfaces__msg__FingerData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hand_interfaces__msg__FingerData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hand_interfaces__msg__FingerData * data =
      (hand_interfaces__msg__FingerData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hand_interfaces__msg__FingerData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hand_interfaces__msg__FingerData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hand_interfaces__msg__FingerData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
