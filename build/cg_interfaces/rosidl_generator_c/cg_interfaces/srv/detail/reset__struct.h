// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cg_interfaces:srv/Reset.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/reset.h"


#ifndef CG_INTERFACES__SRV__DETAIL__RESET__STRUCT_H_
#define CG_INTERFACES__SRV__DETAIL__RESET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'map_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Reset in the package cg_interfaces.
typedef struct cg_interfaces__srv__Reset_Request
{
  bool is_random;
  /// e.g., "1.csv". If empty and is_random is false, resets current map.
  rosidl_runtime_c__String map_name;
} cg_interfaces__srv__Reset_Request;

// Struct for a sequence of cg_interfaces__srv__Reset_Request.
typedef struct cg_interfaces__srv__Reset_Request__Sequence
{
  cg_interfaces__srv__Reset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__Reset_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'loaded_map_name'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Reset in the package cg_interfaces.
typedef struct cg_interfaces__srv__Reset_Response
{
  bool success;
  rosidl_runtime_c__String loaded_map_name;
} cg_interfaces__srv__Reset_Response;

// Struct for a sequence of cg_interfaces__srv__Reset_Response.
typedef struct cg_interfaces__srv__Reset_Response__Sequence
{
  cg_interfaces__srv__Reset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__Reset_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  cg_interfaces__srv__Reset_Event__request__MAX_SIZE = 1
};
// response
enum
{
  cg_interfaces__srv__Reset_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Reset in the package cg_interfaces.
typedef struct cg_interfaces__srv__Reset_Event
{
  service_msgs__msg__ServiceEventInfo info;
  cg_interfaces__srv__Reset_Request__Sequence request;
  cg_interfaces__srv__Reset_Response__Sequence response;
} cg_interfaces__srv__Reset_Event;

// Struct for a sequence of cg_interfaces__srv__Reset_Event.
typedef struct cg_interfaces__srv__Reset_Event__Sequence
{
  cg_interfaces__srv__Reset_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cg_interfaces__srv__Reset_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CG_INTERFACES__SRV__DETAIL__RESET__STRUCT_H_
