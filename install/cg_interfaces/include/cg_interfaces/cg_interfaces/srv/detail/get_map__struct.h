// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/get_map.h"


#ifndef CG_INTERFACES__SRV__DETAIL__GET_MAP__STRUCT_H_
#define CG_INTERFACES__SRV__DETAIL__GET_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetMap in the package cg_interfaces.
typedef struct cg_interfaces__srv__GetMap_Request
{
  uint8_t structure_needs_at_least_one_member;
} cg_interfaces__srv__GetMap_Request;

// Struct for a sequence of cg_interfaces__srv__GetMap_Request.
typedef struct cg_interfaces__srv__GetMap_Request__Sequence
{
  cg_interfaces__srv__GetMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__GetMap_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'occupancy_grid_flattened'
#include "rosidl_runtime_c/string.h"
// Member 'occupancy_grid_shape'
#include "rosidl_runtime_c/primitives_sequence.h"

// constants for array fields with an upper bound
// occupancy_grid_shape
enum
{
  cg_interfaces__srv__GetMap_Response__occupancy_grid_shape__MAX_SIZE = 2
};

/// Struct defined in srv/GetMap in the package cg_interfaces.
typedef struct cg_interfaces__srv__GetMap_Response
{
  rosidl_runtime_c__String__Sequence occupancy_grid_flattened;
  rosidl_runtime_c__uint8__Sequence occupancy_grid_shape;
} cg_interfaces__srv__GetMap_Response;

// Struct for a sequence of cg_interfaces__srv__GetMap_Response.
typedef struct cg_interfaces__srv__GetMap_Response__Sequence
{
  cg_interfaces__srv__GetMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__GetMap_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  cg_interfaces__srv__GetMap_Event__request__MAX_SIZE = 1
};
// response
enum
{
  cg_interfaces__srv__GetMap_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetMap in the package cg_interfaces.
typedef struct cg_interfaces__srv__GetMap_Event
{
  service_msgs__msg__ServiceEventInfo info;
  cg_interfaces__srv__GetMap_Request__Sequence request;
  cg_interfaces__srv__GetMap_Response__Sequence response;
} cg_interfaces__srv__GetMap_Event;

// Struct for a sequence of cg_interfaces__srv__GetMap_Event.
typedef struct cg_interfaces__srv__GetMap_Event__Sequence
{
  cg_interfaces__srv__GetMap_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__GetMap_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CG_INTERFACES__SRV__DETAIL__GET_MAP__STRUCT_H_
