// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice
#ifndef CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "cg_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "cg_interfaces/msg/detail/robot_sensors__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
bool cdr_serialize_cg_interfaces__msg__RobotSensors(
  const cg_interfaces__msg__RobotSensors * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
bool cdr_deserialize_cg_interfaces__msg__RobotSensors(
  eprosima::fastcdr::Cdr &,
  cg_interfaces__msg__RobotSensors * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t get_serialized_size_cg_interfaces__msg__RobotSensors(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t max_serialized_size_cg_interfaces__msg__RobotSensors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
bool cdr_serialize_key_cg_interfaces__msg__RobotSensors(
  const cg_interfaces__msg__RobotSensors * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t get_serialized_size_key_cg_interfaces__msg__RobotSensors(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t max_serialized_size_key_cg_interfaces__msg__RobotSensors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cg_interfaces, msg, RobotSensors)();

#ifdef __cplusplus
}
#endif

#endif  // CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
