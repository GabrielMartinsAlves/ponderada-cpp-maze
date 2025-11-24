// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/msg/robot_sensors.h"


#ifndef CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__STRUCT_H_
#define CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'up'
// Member 'down'
// Member 'left'
// Member 'right'
// Member 'up_left'
// Member 'up_right'
// Member 'down_left'
// Member 'down_right'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotSensors in the package cg_interfaces.
typedef struct cg_interfaces__msg__RobotSensors
{
  rosidl_runtime_c__String up;
  rosidl_runtime_c__String down;
  rosidl_runtime_c__String left;
  rosidl_runtime_c__String right;
  rosidl_runtime_c__String up_left;
  rosidl_runtime_c__String up_right;
  rosidl_runtime_c__String down_left;
  rosidl_runtime_c__String down_right;
} cg_interfaces__msg__RobotSensors;

// Struct for a sequence of cg_interfaces__msg__RobotSensors.
typedef struct cg_interfaces__msg__RobotSensors__Sequence
{
  cg_interfaces__msg__RobotSensors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__msg__RobotSensors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__STRUCT_H_
