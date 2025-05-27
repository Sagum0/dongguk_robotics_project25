// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotics_interfaces:srv/MotorExecutor.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__STRUCT_H_
#define ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorExecutor in the package robotics_interfaces.
typedef struct robotics_interfaces__srv__MotorExecutor_Request
{
  float x;
  float y;
  float z;
  float r;
  bool grab;
  rosidl_runtime_c__String task;
  float time;
} robotics_interfaces__srv__MotorExecutor_Request;

// Struct for a sequence of robotics_interfaces__srv__MotorExecutor_Request.
typedef struct robotics_interfaces__srv__MotorExecutor_Request__Sequence
{
  robotics_interfaces__srv__MotorExecutor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotics_interfaces__srv__MotorExecutor_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MotorExecutor in the package robotics_interfaces.
typedef struct robotics_interfaces__srv__MotorExecutor_Response
{
  bool success;
} robotics_interfaces__srv__MotorExecutor_Response;

// Struct for a sequence of robotics_interfaces__srv__MotorExecutor_Response.
typedef struct robotics_interfaces__srv__MotorExecutor_Response__Sequence
{
  robotics_interfaces__srv__MotorExecutor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotics_interfaces__srv__MotorExecutor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__STRUCT_H_
