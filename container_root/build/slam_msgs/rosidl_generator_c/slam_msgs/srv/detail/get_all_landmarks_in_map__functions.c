// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from slam_msgs:srv/GetAllLandmarksInMap.idl
// generated code does not contain a copyright notice
#include "slam_msgs/srv/detail/get_all_landmarks_in_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
slam_msgs__srv__GetAllLandmarksInMap_Request__init(slam_msgs__srv__GetAllLandmarksInMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // request
  msg->request = true;
  return true;
}

void
slam_msgs__srv__GetAllLandmarksInMap_Request__fini(slam_msgs__srv__GetAllLandmarksInMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // request
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Request__are_equal(const slam_msgs__srv__GetAllLandmarksInMap_Request * lhs, const slam_msgs__srv__GetAllLandmarksInMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request
  if (lhs->request != rhs->request) {
    return false;
  }
  return true;
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Request__copy(
  const slam_msgs__srv__GetAllLandmarksInMap_Request * input,
  slam_msgs__srv__GetAllLandmarksInMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // request
  output->request = input->request;
  return true;
}

slam_msgs__srv__GetAllLandmarksInMap_Request *
slam_msgs__srv__GetAllLandmarksInMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__srv__GetAllLandmarksInMap_Request * msg = (slam_msgs__srv__GetAllLandmarksInMap_Request *)allocator.allocate(sizeof(slam_msgs__srv__GetAllLandmarksInMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(slam_msgs__srv__GetAllLandmarksInMap_Request));
  bool success = slam_msgs__srv__GetAllLandmarksInMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
slam_msgs__srv__GetAllLandmarksInMap_Request__destroy(slam_msgs__srv__GetAllLandmarksInMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    slam_msgs__srv__GetAllLandmarksInMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__init(slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__srv__GetAllLandmarksInMap_Request * data = NULL;

  if (size) {
    data = (slam_msgs__srv__GetAllLandmarksInMap_Request *)allocator.zero_allocate(size, sizeof(slam_msgs__srv__GetAllLandmarksInMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = slam_msgs__srv__GetAllLandmarksInMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        slam_msgs__srv__GetAllLandmarksInMap_Request__fini(&data[i - 1]);
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
slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__fini(slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * array)
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
      slam_msgs__srv__GetAllLandmarksInMap_Request__fini(&array->data[i]);
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

slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence *
slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * array = (slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence *)allocator.allocate(sizeof(slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__destroy(slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__are_equal(const slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * lhs, const slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!slam_msgs__srv__GetAllLandmarksInMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence__copy(
  const slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * input,
  slam_msgs__srv__GetAllLandmarksInMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(slam_msgs__srv__GetAllLandmarksInMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    slam_msgs__srv__GetAllLandmarksInMap_Request * data =
      (slam_msgs__srv__GetAllLandmarksInMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!slam_msgs__srv__GetAllLandmarksInMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          slam_msgs__srv__GetAllLandmarksInMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!slam_msgs__srv__GetAllLandmarksInMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `landmarks`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
slam_msgs__srv__GetAllLandmarksInMap_Response__init(slam_msgs__srv__GetAllLandmarksInMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // landmarks
  if (!sensor_msgs__msg__PointCloud2__init(&msg->landmarks)) {
    slam_msgs__srv__GetAllLandmarksInMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
slam_msgs__srv__GetAllLandmarksInMap_Response__fini(slam_msgs__srv__GetAllLandmarksInMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // landmarks
  sensor_msgs__msg__PointCloud2__fini(&msg->landmarks);
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Response__are_equal(const slam_msgs__srv__GetAllLandmarksInMap_Response * lhs, const slam_msgs__srv__GetAllLandmarksInMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // landmarks
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->landmarks), &(rhs->landmarks)))
  {
    return false;
  }
  return true;
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Response__copy(
  const slam_msgs__srv__GetAllLandmarksInMap_Response * input,
  slam_msgs__srv__GetAllLandmarksInMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // landmarks
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->landmarks), &(output->landmarks)))
  {
    return false;
  }
  return true;
}

slam_msgs__srv__GetAllLandmarksInMap_Response *
slam_msgs__srv__GetAllLandmarksInMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__srv__GetAllLandmarksInMap_Response * msg = (slam_msgs__srv__GetAllLandmarksInMap_Response *)allocator.allocate(sizeof(slam_msgs__srv__GetAllLandmarksInMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(slam_msgs__srv__GetAllLandmarksInMap_Response));
  bool success = slam_msgs__srv__GetAllLandmarksInMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
slam_msgs__srv__GetAllLandmarksInMap_Response__destroy(slam_msgs__srv__GetAllLandmarksInMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    slam_msgs__srv__GetAllLandmarksInMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__init(slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__srv__GetAllLandmarksInMap_Response * data = NULL;

  if (size) {
    data = (slam_msgs__srv__GetAllLandmarksInMap_Response *)allocator.zero_allocate(size, sizeof(slam_msgs__srv__GetAllLandmarksInMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = slam_msgs__srv__GetAllLandmarksInMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        slam_msgs__srv__GetAllLandmarksInMap_Response__fini(&data[i - 1]);
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
slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__fini(slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * array)
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
      slam_msgs__srv__GetAllLandmarksInMap_Response__fini(&array->data[i]);
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

slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence *
slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * array = (slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence *)allocator.allocate(sizeof(slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__destroy(slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__are_equal(const slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * lhs, const slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!slam_msgs__srv__GetAllLandmarksInMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence__copy(
  const slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * input,
  slam_msgs__srv__GetAllLandmarksInMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(slam_msgs__srv__GetAllLandmarksInMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    slam_msgs__srv__GetAllLandmarksInMap_Response * data =
      (slam_msgs__srv__GetAllLandmarksInMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!slam_msgs__srv__GetAllLandmarksInMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          slam_msgs__srv__GetAllLandmarksInMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!slam_msgs__srv__GetAllLandmarksInMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
