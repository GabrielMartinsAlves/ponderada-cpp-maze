// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/move_cmd.h"


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

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  cg_interfaces__srv__MoveCmd_Event__request__MAX_SIZE = 1
};
// response
enum
{
  cg_interfaces__srv__MoveCmd_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveCmd in the package cg_interfaces.
typedef struct cg_interfaces__srv__MoveCmd_Event
{
  service_msgs__msg__ServiceEventInfo info;
  cg_interfaces__srv__MoveCmd_Request__Sequence request;
  cg_interfaces__srv__MoveCmd_Response__Sequence response;
} cg_interfaces__srv__MoveCmd_Event;

// Struct for a sequence of cg_interfaces__srv__MoveCmd_Event.
typedef struct cg_interfaces__srv__MoveCmd_Event__Sequence
{
  cg_interfaces__srv__MoveCmd_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__MoveCmd_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_H_
