// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotics_interfaces:msg/TrajectoryPoints.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__STRUCT_H_
#define ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/TrajectoryPoints in the package robotics_interfaces.
/**
  * msg/TrajectoryPoints.msg
 */
typedef struct robotics_interfaces__msg__TrajectoryPoints
{
  geometry_msgs__msg__Point__Sequence points;
} robotics_interfaces__msg__TrajectoryPoints;

// Struct for a sequence of robotics_interfaces__msg__TrajectoryPoints.
typedef struct robotics_interfaces__msg__TrajectoryPoints__Sequence
{
  robotics_interfaces__msg__TrajectoryPoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotics_interfaces__msg__TrajectoryPoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__STRUCT_H_
