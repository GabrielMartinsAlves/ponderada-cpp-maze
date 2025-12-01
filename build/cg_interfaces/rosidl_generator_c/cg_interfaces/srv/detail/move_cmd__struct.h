// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice

#ifndef CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_H_
#define CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'direction'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveCmd in the package cg_interfaces.
typedef struct cg_interfaces__srv__MoveCmd_Request
{
  rosidl_runtime_c__String direction;
} cg_interfaces__srv__MoveCmd_Request;

// Struct for a sequence of cg_interfaces__srv__MoveCmd_Request.
typedef struct cg_interfaces__srv__MoveCmd_Request__Sequence
{
  cg_interfaces__srv__MoveCmd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__MoveCmd_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MoveCmd in the package cg_interfaces.
typedef struct cg_interfaces__srv__MoveCmd_Response
{
  bool success;
  int8_t robot_pos[2];
  int8_t target_pos[2];
} cg_interfaces__srv__MoveCmd_Response;

// Struct for a sequence of cg_interfaces__srv__MoveCmd_Response.
typedef struct cg_interfaces__srv__MoveCmd_Response__Sequence
{
  cg_interfaces__srv__MoveCmd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__MoveCmd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_H_
