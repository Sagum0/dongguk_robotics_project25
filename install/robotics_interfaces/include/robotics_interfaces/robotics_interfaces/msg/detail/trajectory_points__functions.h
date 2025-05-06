// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotics_interfaces:msg/TrajectoryPoints.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__FUNCTIONS_H_
#define ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotics_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "robotics_interfaces/msg/detail/trajectory_points__struct.h"

/// Initialize msg/TrajectoryPoints message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotics_interfaces__msg__TrajectoryPoints
 * )) before or use
 * robotics_interfaces__msg__TrajectoryPoints__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
bool
robotics_interfaces__msg__TrajectoryPoints__init(robotics_interfaces__msg__TrajectoryPoints * msg);

/// Finalize msg/TrajectoryPoints message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
void
robotics_interfaces__msg__TrajectoryPoints__fini(robotics_interfaces__msg__TrajectoryPoints * msg);

/// Create msg/TrajectoryPoints message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotics_interfaces__msg__TrajectoryPoints__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
robotics_interfaces__msg__TrajectoryPoints *
robotics_interfaces__msg__TrajectoryPoints__create();

/// Destroy msg/TrajectoryPoints message.
/**
 * It calls
 * robotics_interfaces__msg__TrajectoryPoints__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
void
robotics_interfaces__msg__TrajectoryPoints__destroy(robotics_interfaces__msg__TrajectoryPoints * msg);

/// Check for msg/TrajectoryPoints message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
bool
robotics_interfaces__msg__TrajectoryPoints__are_equal(const robotics_interfaces__msg__TrajectoryPoints * lhs, const robotics_interfaces__msg__TrajectoryPoints * rhs);

/// Copy a msg/TrajectoryPoints message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
bool
robotics_interfaces__msg__TrajectoryPoints__copy(
  const robotics_interfaces__msg__TrajectoryPoints * input,
  robotics_interfaces__msg__TrajectoryPoints * output);

/// Initialize array of msg/TrajectoryPoints messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotics_interfaces__msg__TrajectoryPoints__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
bool
robotics_interfaces__msg__TrajectoryPoints__Sequence__init(robotics_interfaces__msg__TrajectoryPoints__Sequence * array, size_t size);

/// Finalize array of msg/TrajectoryPoints messages.
/**
 * It calls
 * robotics_interfaces__msg__TrajectoryPoints__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
void
robotics_interfaces__msg__TrajectoryPoints__Sequence__fini(robotics_interfaces__msg__TrajectoryPoints__Sequence * array);

/// Create array of msg/TrajectoryPoints messages.
/**
 * It allocates the memory for the array and calls
 * robotics_interfaces__msg__TrajectoryPoints__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
robotics_interfaces__msg__TrajectoryPoints__Sequence *
robotics_interfaces__msg__TrajectoryPoints__Sequence__create(size_t size);

/// Destroy array of msg/TrajectoryPoints messages.
/**
 * It calls
 * robotics_interfaces__msg__TrajectoryPoints__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
void
robotics_interfaces__msg__TrajectoryPoints__Sequence__destroy(robotics_interfaces__msg__TrajectoryPoints__Sequence * array);

/// Check for msg/TrajectoryPoints message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
bool
robotics_interfaces__msg__TrajectoryPoints__Sequence__are_equal(const robotics_interfaces__msg__TrajectoryPoints__Sequence * lhs, const robotics_interfaces__msg__TrajectoryPoints__Sequence * rhs);

/// Copy an array of msg/TrajectoryPoints messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotics_interfaces
bool
robotics_interfaces__msg__TrajectoryPoints__Sequence__copy(
  const robotics_interfaces__msg__TrajectoryPoints__Sequence * input,
  robotics_interfaces__msg__TrajectoryPoints__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__FUNCTIONS_H_
