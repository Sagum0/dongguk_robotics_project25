// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotics_interfaces:srv/MotorExecutor.idl
// generated code does not contain a copyright notice
#include "robotics_interfaces/srv/detail/motor_executor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `task`
#include "rosidl_runtime_c/string_functions.h"

bool
robotics_interfaces__srv__MotorExecutor_Request__init(robotics_interfaces__srv__MotorExecutor_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // r
  // grab
  // task
  if (!rosidl_runtime_c__String__init(&msg->task)) {
    robotics_interfaces__srv__MotorExecutor_Request__fini(msg);
    return false;
  }
  // time
  return true;
}

void
robotics_interfaces__srv__MotorExecutor_Request__fini(robotics_interfaces__srv__MotorExecutor_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // r
  // grab
  // task
  rosidl_runtime_c__String__fini(&msg->task);
  // time
}

bool
robotics_interfaces__srv__MotorExecutor_Request__are_equal(const robotics_interfaces__srv__MotorExecutor_Request * lhs, const robotics_interfaces__srv__MotorExecutor_Request * rhs)
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
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  // grab
  if (lhs->grab != rhs->grab) {
    return false;
  }
  // task
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task), &(rhs->task)))
  {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  return true;
}

bool
robotics_interfaces__srv__MotorExecutor_Request__copy(
  const robotics_interfaces__srv__MotorExecutor_Request * input,
  robotics_interfaces__srv__MotorExecutor_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // r
  output->r = input->r;
  // grab
  output->grab = input->grab;
  // task
  if (!rosidl_runtime_c__String__copy(
      &(input->task), &(output->task)))
  {
    return false;
  }
  // time
  output->time = input->time;
  return true;
}

robotics_interfaces__srv__MotorExecutor_Request *
robotics_interfaces__srv__MotorExecutor_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__srv__MotorExecutor_Request * msg = (robotics_interfaces__srv__MotorExecutor_Request *)allocator.allocate(sizeof(robotics_interfaces__srv__MotorExecutor_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotics_interfaces__srv__MotorExecutor_Request));
  bool success = robotics_interfaces__srv__MotorExecutor_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotics_interfaces__srv__MotorExecutor_Request__destroy(robotics_interfaces__srv__MotorExecutor_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotics_interfaces__srv__MotorExecutor_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotics_interfaces__srv__MotorExecutor_Request__Sequence__init(robotics_interfaces__srv__MotorExecutor_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__srv__MotorExecutor_Request * data = NULL;

  if (size) {
    data = (robotics_interfaces__srv__MotorExecutor_Request *)allocator.zero_allocate(size, sizeof(robotics_interfaces__srv__MotorExecutor_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotics_interfaces__srv__MotorExecutor_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotics_interfaces__srv__MotorExecutor_Request__fini(&data[i - 1]);
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
robotics_interfaces__srv__MotorExecutor_Request__Sequence__fini(robotics_interfaces__srv__MotorExecutor_Request__Sequence * array)
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
      robotics_interfaces__srv__MotorExecutor_Request__fini(&array->data[i]);
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

robotics_interfaces__srv__MotorExecutor_Request__Sequence *
robotics_interfaces__srv__MotorExecutor_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__srv__MotorExecutor_Request__Sequence * array = (robotics_interfaces__srv__MotorExecutor_Request__Sequence *)allocator.allocate(sizeof(robotics_interfaces__srv__MotorExecutor_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotics_interfaces__srv__MotorExecutor_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotics_interfaces__srv__MotorExecutor_Request__Sequence__destroy(robotics_interfaces__srv__MotorExecutor_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotics_interfaces__srv__MotorExecutor_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotics_interfaces__srv__MotorExecutor_Request__Sequence__are_equal(const robotics_interfaces__srv__MotorExecutor_Request__Sequence * lhs, const robotics_interfaces__srv__MotorExecutor_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotics_interfaces__srv__MotorExecutor_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotics_interfaces__srv__MotorExecutor_Request__Sequence__copy(
  const robotics_interfaces__srv__MotorExecutor_Request__Sequence * input,
  robotics_interfaces__srv__MotorExecutor_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotics_interfaces__srv__MotorExecutor_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotics_interfaces__srv__MotorExecutor_Request * data =
      (robotics_interfaces__srv__MotorExecutor_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotics_interfaces__srv__MotorExecutor_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotics_interfaces__srv__MotorExecutor_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotics_interfaces__srv__MotorExecutor_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
robotics_interfaces__srv__MotorExecutor_Response__init(robotics_interfaces__srv__MotorExecutor_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
robotics_interfaces__srv__MotorExecutor_Response__fini(robotics_interfaces__srv__MotorExecutor_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
robotics_interfaces__srv__MotorExecutor_Response__are_equal(const robotics_interfaces__srv__MotorExecutor_Response * lhs, const robotics_interfaces__srv__MotorExecutor_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
robotics_interfaces__srv__MotorExecutor_Response__copy(
  const robotics_interfaces__srv__MotorExecutor_Response * input,
  robotics_interfaces__srv__MotorExecutor_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

robotics_interfaces__srv__MotorExecutor_Response *
robotics_interfaces__srv__MotorExecutor_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__srv__MotorExecutor_Response * msg = (robotics_interfaces__srv__MotorExecutor_Response *)allocator.allocate(sizeof(robotics_interfaces__srv__MotorExecutor_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotics_interfaces__srv__MotorExecutor_Response));
  bool success = robotics_interfaces__srv__MotorExecutor_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotics_interfaces__srv__MotorExecutor_Response__destroy(robotics_interfaces__srv__MotorExecutor_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotics_interfaces__srv__MotorExecutor_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotics_interfaces__srv__MotorExecutor_Response__Sequence__init(robotics_interfaces__srv__MotorExecutor_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__srv__MotorExecutor_Response * data = NULL;

  if (size) {
    data = (robotics_interfaces__srv__MotorExecutor_Response *)allocator.zero_allocate(size, sizeof(robotics_interfaces__srv__MotorExecutor_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotics_interfaces__srv__MotorExecutor_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotics_interfaces__srv__MotorExecutor_Response__fini(&data[i - 1]);
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
robotics_interfaces__srv__MotorExecutor_Response__Sequence__fini(robotics_interfaces__srv__MotorExecutor_Response__Sequence * array)
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
      robotics_interfaces__srv__MotorExecutor_Response__fini(&array->data[i]);
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

robotics_interfaces__srv__MotorExecutor_Response__Sequence *
robotics_interfaces__srv__MotorExecutor_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotics_interfaces__srv__MotorExecutor_Response__Sequence * array = (robotics_interfaces__srv__MotorExecutor_Response__Sequence *)allocator.allocate(sizeof(robotics_interfaces__srv__MotorExecutor_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotics_interfaces__srv__MotorExecutor_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotics_interfaces__srv__MotorExecutor_Response__Sequence__destroy(robotics_interfaces__srv__MotorExecutor_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotics_interfaces__srv__MotorExecutor_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotics_interfaces__srv__MotorExecutor_Response__Sequence__are_equal(const robotics_interfaces__srv__MotorExecutor_Response__Sequence * lhs, const robotics_interfaces__srv__MotorExecutor_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotics_interfaces__srv__MotorExecutor_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotics_interfaces__srv__MotorExecutor_Response__Sequence__copy(
  const robotics_interfaces__srv__MotorExecutor_Response__Sequence * input,
  robotics_interfaces__srv__MotorExecutor_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotics_interfaces__srv__MotorExecutor_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotics_interfaces__srv__MotorExecutor_Response * data =
      (robotics_interfaces__srv__MotorExecutor_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotics_interfaces__srv__MotorExecutor_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotics_interfaces__srv__MotorExecutor_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotics_interfaces__srv__MotorExecutor_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
