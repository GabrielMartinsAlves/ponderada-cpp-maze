// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cg_interfaces/msg/detail/robot_sensors__rosidl_typesupport_introspection_c.h"
#include "cg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cg_interfaces/msg/detail/robot_sensors__functions.h"
#include "cg_interfaces/msg/detail/robot_sensors__struct.h"


// Include directives for member types
// Member `up`
// Member `down`
// Member `left`
// Member `right`
// Member `up_left`
// Member `up_right`
// Member `down_left`
// Member `down_right`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cg_interfaces__msg__RobotSensors__init(message_memory);
}

void cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_fini_function(void * message_memory)
{
  cg_interfaces__msg__RobotSensors__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_member_array[8] = {
  {
    "up",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, up),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "down",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, down),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "up_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, up_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "up_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, up_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "down_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, down_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "down_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces__msg__RobotSensors, down_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_members = {
  "cg_interfaces__msg",  // message namespace
  "RobotSensors",  // message name
  8,  // number of fields
  sizeof(cg_interfaces__msg__RobotSensors),
  false,  // has_any_key_member_
  cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_member_array,  // message members
  cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_init_function,  // function to initialize message memory (memory has to be allocated)
  cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_type_support_handle = {
  0,
  &cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_members,
  get_message_typesupport_handle_function,
  &cg_interfaces__msg__RobotSensors__get_type_hash,
  &cg_interfaces__msg__RobotSensors__get_type_description,
  &cg_interfaces__msg__RobotSensors__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cg_interfaces, msg, RobotSensors)() {
  if (!cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_type_support_handle.typesupport_identifier) {
    cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cg_interfaces__msg__RobotSensors__rosidl_typesupport_introspection_c__RobotSensors_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
