// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cg_interfaces/srv/detail/get_map__rosidl_typesupport_introspection_c.h"
#include "cg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cg_interfaces/srv/detail/get_map__functions.h"
#include "cg_interfaces/srv/detail/get_map__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cg_interfaces__srv__GetMap_Request__init(message_memory);
}

void cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_fini_function(void * message_memory)
{
  cg_interfaces__srv__GetMap_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__srv__GetMap_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_members = {
  "cg_interfaces__srv",  // message namespace
  "GetMap_Request",  // message name
  1,  // number of fields
  sizeof(cg_interfaces__srv__GetMap_Request),
  cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_member_array,  // message members
  cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_type_support_handle = {
  0,
  &cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap_Request)() {
  if (!cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_type_support_handle.typesupport_identifier) {
    cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cg_interfaces__srv__GetMap_Request__rosidl_typesupport_introspection_c__GetMap_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "cg_interfaces/srv/detail/get_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "cg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__functions.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__struct.h"


// Include directives for member types
// Member `occupancy_grid_flattened`
#include "rosidl_runtime_c/string_functions.h"
// Member `occupancy_grid_shape`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cg_interfaces__srv__GetMap_Response__init(message_memory);
}

void cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_fini_function(void * message_memory)
{
  cg_interfaces__srv__GetMap_Response__fini(message_memory);
}

size_t cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__size_function__GetMap_Response__occupancy_grid_flattened(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetMap_Response__occupancy_grid_flattened(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_function__GetMap_Response__occupancy_grid_flattened(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__fetch_function__GetMap_Response__occupancy_grid_flattened(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetMap_Response__occupancy_grid_flattened(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__assign_function__GetMap_Response__occupancy_grid_flattened(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_function__GetMap_Response__occupancy_grid_flattened(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__resize_function__GetMap_Response__occupancy_grid_flattened(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__size_function__GetMap_Response__occupancy_grid_shape(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetMap_Response__occupancy_grid_shape(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_function__GetMap_Response__occupancy_grid_shape(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__fetch_function__GetMap_Response__occupancy_grid_shape(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetMap_Response__occupancy_grid_shape(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__assign_function__GetMap_Response__occupancy_grid_shape(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_function__GetMap_Response__occupancy_grid_shape(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__resize_function__GetMap_Response__occupancy_grid_shape(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_member_array[2] = {
  {
    "occupancy_grid_flattened",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__srv__GetMap_Response, occupancy_grid_flattened),  // bytes offset in struct
    NULL,  // default value
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__size_function__GetMap_Response__occupancy_grid_flattened,  // size() function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetMap_Response__occupancy_grid_flattened,  // get_const(index) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_function__GetMap_Response__occupancy_grid_flattened,  // get(index) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__fetch_function__GetMap_Response__occupancy_grid_flattened,  // fetch(index, &value) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__assign_function__GetMap_Response__occupancy_grid_flattened,  // assign(index, value) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__resize_function__GetMap_Response__occupancy_grid_flattened  // resize(index) function pointer
  },
  {
    "occupancy_grid_shape",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    true,  // is upper bound
    offsetof(cg_interfaces__srv__GetMap_Response, occupancy_grid_shape),  // bytes offset in struct
    NULL,  // default value
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__size_function__GetMap_Response__occupancy_grid_shape,  // size() function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_const_function__GetMap_Response__occupancy_grid_shape,  // get_const(index) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__get_function__GetMap_Response__occupancy_grid_shape,  // get(index) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__fetch_function__GetMap_Response__occupancy_grid_shape,  // fetch(index, &value) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__assign_function__GetMap_Response__occupancy_grid_shape,  // assign(index, value) function pointer
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__resize_function__GetMap_Response__occupancy_grid_shape  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_members = {
  "cg_interfaces__srv",  // message namespace
  "GetMap_Response",  // message name
  2,  // number of fields
  sizeof(cg_interfaces__srv__GetMap_Response),
  cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_member_array,  // message members
  cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_type_support_handle = {
  0,
  &cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap_Response)() {
  if (!cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_type_support_handle.typesupport_identifier) {
    cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cg_interfaces__srv__GetMap_Response__rosidl_typesupport_introspection_c__GetMap_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "cg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_members = {
  "cg_interfaces__srv",  // service namespace
  "GetMap",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_Request_message_type_support_handle,
  NULL  // response message
  // cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_Response_message_type_support_handle
};

static rosidl_service_type_support_t cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_type_support_handle = {
  0,
  &cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cg_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap)() {
  if (!cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_type_support_handle.typesupport_identifier) {
    cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, srv, GetMap_Response)()->data;
  }

  return &cg_interfaces__srv__detail__get_map__rosidl_typesupport_introspection_c__GetMap_service_type_support_handle;
}
