// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotics_interfaces:msg/TrajectoryPoints.idl
// generated code does not contain a copyright notice
#include "robotics_interfaces/msg/detail/trajectory_points__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `points`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
robotics_interfaces__msg__TrajectoryPoints__init(robotics_interfaces__msg__TrajectoryPoints * msg)
{
  if (!msg) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    robotics_interfaces__msg__TrajectoryPoints__fini(msg);
    return false;
  }
  return true;
}

void
robotics_interfaces__msg__TrajectoryPoints__fini(robotics_interfaces__msg__TrajectoryPoints * msg)
{
  if (!msg) {
    return;
  }
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
}

bool
robotics_interfaces__msg__TrajectoryPoints__are_equal(const robotics_interfaces__msg__TrajectoryPoints * lhs, const robotics_interfaces__msg__TrajectoryPoints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  return true;
}

bool
robotics_interfaces__msg__TrajectoryPoints__copy(
  const robotics_interfaces__msg__TrajectoryPoints * input,
  robotics_interfaces__msg__TrajectoryPoints * output)
{
  if (!input || !output) {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  return true;
}

robotics_interfaces__msg__TrajectoryPoints *
robotics_interfaces__msg__TrajectoryPoints__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__msg__TrajectoryPoints * msg = (robotics_interfaces__msg__TrajectoryPoints *)allocator.allocate(sizeof(robotics_interfaces__msg__TrajectoryPoints), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotics_interfaces__msg__TrajectoryPoints));
  bool success = robotics_interfaces__msg__TrajectoryPoints__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotics_interfaces__msg__TrajectoryPoints__destroy(robotics_interfaces__msg__TrajectoryPoints * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotics_interfaces__msg__TrajectoryPoints__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotics_interfaces__msg__TrajectoryPoints__Sequence__init(robotics_interfaces__msg__TrajectoryPoints__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__msg__TrajectoryPoints * data = NULL;

  if (size) {
    data = (robotics_interfaces__msg__TrajectoryPoints *)allocator.zero_allocate(size, sizeof(robotics_interfaces__msg__TrajectoryPoints), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotics_interfaces__msg__TrajectoryPoints__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotics_interfaces__msg__TrajectoryPoints__fini(&data[i - 1]);
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
robotics_interfaces__msg__TrajectoryPoints__Sequence__fini(robotics_interfaces__msg__TrajectoryPoints__Sequence * array)
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
      robotics_interfaces__msg__TrajectoryPoints__fini(&array->data[i]);
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

robotics_interfaces__msg__TrajectoryPoints__Sequence *
robotics_interfaces__msg__TrajectoryPoints__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__msg__TrajectoryPoints__Sequence * array = (robotics_interfaces__msg__TrajectoryPoints__Sequence *)allocator.allocate(sizeof(robotics_interfaces__msg__TrajectoryPoints__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotics_interfaces__msg__TrajectoryPoints__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotics_interfaces__msg__TrajectoryPoints__Sequence__destroy(robotics_interfaces__msg__TrajectoryPoints__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotics_interfaces__msg__TrajectoryPoints__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotics_interfaces__msg__TrajectoryPoints__Sequence__are_equal(const robotics_interfaces__msg__TrajectoryPoints__Sequence * lhs, const robotics_interfaces__msg__TrajectoryPoints__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotics_interfaces__msg__TrajectoryPoints__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotics_interfaces__msg__TrajectoryPoints__Sequence__copy(
  const robotics_interfaces__msg__TrajectoryPoints__Sequence * input,
  robotics_interfaces__msg__TrajectoryPoints__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotics_interfaces__msg__TrajectoryPoints);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotics_interfaces__msg__TrajectoryPoints * data =
      (robotics_interfaces__msg__TrajectoryPoints *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotics_interfaces__msg__TrajectoryPoints__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotics_interfaces__msg__TrajectoryPoints__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotics_interfaces__msg__TrajectoryPoints__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
