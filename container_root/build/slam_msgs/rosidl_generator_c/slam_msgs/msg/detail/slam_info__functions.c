// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from slam_msgs:msg/SlamInfo.idl
// generated code does not contain a copyright notice
#include "slam_msgs/msg/detail/slam_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
slam_msgs__msg__SlamInfo__init(slam_msgs__msg__SlamInfo * msg)
{
  if (!msg) {
    return false;
  }
  // num_maps
  // num_keyframes_in_current_map
  // tracking_frequency
  return true;
}

void
slam_msgs__msg__SlamInfo__fini(slam_msgs__msg__SlamInfo * msg)
{
  if (!msg) {
    return;
  }
  // num_maps
  // num_keyframes_in_current_map
  // tracking_frequency
}

bool
slam_msgs__msg__SlamInfo__are_equal(const slam_msgs__msg__SlamInfo * lhs, const slam_msgs__msg__SlamInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num_maps
  if (lhs->num_maps != rhs->num_maps) {
    return false;
  }
  // num_keyframes_in_current_map
  if (lhs->num_keyframes_in_current_map != rhs->num_keyframes_in_current_map) {
    return false;
  }
  // tracking_frequency
  if (lhs->tracking_frequency != rhs->tracking_frequency) {
    return false;
  }
  return true;
}

bool
slam_msgs__msg__SlamInfo__copy(
  const slam_msgs__msg__SlamInfo * input,
  slam_msgs__msg__SlamInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // num_maps
  output->num_maps = input->num_maps;
  // num_keyframes_in_current_map
  output->num_keyframes_in_current_map = input->num_keyframes_in_current_map;
  // tracking_frequency
  output->tracking_frequency = input->tracking_frequency;
  return true;
}

slam_msgs__msg__SlamInfo *
slam_msgs__msg__SlamInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__msg__SlamInfo * msg = (slam_msgs__msg__SlamInfo *)allocator.allocate(sizeof(slam_msgs__msg__SlamInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(slam_msgs__msg__SlamInfo));
  bool success = slam_msgs__msg__SlamInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
slam_msgs__msg__SlamInfo__destroy(slam_msgs__msg__SlamInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    slam_msgs__msg__SlamInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
slam_msgs__msg__SlamInfo__Sequence__init(slam_msgs__msg__SlamInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__msg__SlamInfo * data = NULL;

  if (size) {
    data = (slam_msgs__msg__SlamInfo *)allocator.zero_allocate(size, sizeof(slam_msgs__msg__SlamInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = slam_msgs__msg__SlamInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        slam_msgs__msg__SlamInfo__fini(&data[i - 1]);
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
slam_msgs__msg__SlamInfo__Sequence__fini(slam_msgs__msg__SlamInfo__Sequence * array)
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
      slam_msgs__msg__SlamInfo__fini(&array->data[i]);
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

slam_msgs__msg__SlamInfo__Sequence *
slam_msgs__msg__SlamInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__msg__SlamInfo__Sequence * array = (slam_msgs__msg__SlamInfo__Sequence *)allocator.allocate(sizeof(slam_msgs__msg__SlamInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = slam_msgs__msg__SlamInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
slam_msgs__msg__SlamInfo__Sequence__destroy(slam_msgs__msg__SlamInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    slam_msgs__msg__SlamInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
slam_msgs__msg__SlamInfo__Sequence__are_equal(const slam_msgs__msg__SlamInfo__Sequence * lhs, const slam_msgs__msg__SlamInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!slam_msgs__msg__SlamInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
slam_msgs__msg__SlamInfo__Sequence__copy(
  const slam_msgs__msg__SlamInfo__Sequence * input,
  slam_msgs__msg__SlamInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(slam_msgs__msg__SlamInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    slam_msgs__msg__SlamInfo * data =
      (slam_msgs__msg__SlamInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!slam_msgs__msg__SlamInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          slam_msgs__msg__SlamInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!slam_msgs__msg__SlamInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
