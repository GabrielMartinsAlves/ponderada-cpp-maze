// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice
#include "cg_interfaces/msg/detail/robot_sensors__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "cg_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "cg_interfaces/msg/detail/robot_sensors__struct.h"
#include "cg_interfaces/msg/detail/robot_sensors__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // down, down_left, down_right, left, right, up, up_left, up_right
#include "rosidl_runtime_c/string_functions.h"  // down, down_left, down_right, left, right, up, up_left, up_right

// forward declare type support functions


using _RobotSensors__ros_msg_type = cg_interfaces__msg__RobotSensors;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
bool cdr_serialize_cg_interfaces__msg__RobotSensors(
  const cg_interfaces__msg__RobotSensors * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: up
  {
    const rosidl_runtime_c__String * str = &ros_message->up;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: down
  {
    const rosidl_runtime_c__String * str = &ros_message->down;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: left
  {
    const rosidl_runtime_c__String * str = &ros_message->left;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: right
  {
    const rosidl_runtime_c__String * str = &ros_message->right;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: up_left
  {
    const rosidl_runtime_c__String * str = &ros_message->up_left;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: up_right
  {
    const rosidl_runtime_c__String * str = &ros_message->up_right;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: down_left
  {
    const rosidl_runtime_c__String * str = &ros_message->down_left;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: down_right
  {
    const rosidl_runtime_c__String * str = &ros_message->down_right;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
bool cdr_deserialize_cg_interfaces__msg__RobotSensors(
  eprosima::fastcdr::Cdr & cdr,
  cg_interfaces__msg__RobotSensors * ros_message)
{
  // Field name: up
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->up.data) {
      rosidl_runtime_c__String__init(&ros_message->up);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->up,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'up'\n");
      return false;
    }
  }

  // Field name: down
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->down.data) {
      rosidl_runtime_c__String__init(&ros_message->down);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->down,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'down'\n");
      return false;
    }
  }

  // Field name: left
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->left.data) {
      rosidl_runtime_c__String__init(&ros_message->left);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->left,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'left'\n");
      return false;
    }
  }

  // Field name: right
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->right.data) {
      rosidl_runtime_c__String__init(&ros_message->right);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->right,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'right'\n");
      return false;
    }
  }

  // Field name: up_left
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->up_left.data) {
      rosidl_runtime_c__String__init(&ros_message->up_left);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->up_left,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'up_left'\n");
      return false;
    }
  }

  // Field name: up_right
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->up_right.data) {
      rosidl_runtime_c__String__init(&ros_message->up_right);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->up_right,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'up_right'\n");
      return false;
    }
  }

  // Field name: down_left
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->down_left.data) {
      rosidl_runtime_c__String__init(&ros_message->down_left);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->down_left,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'down_left'\n");
      return false;
    }
  }

  // Field name: down_right
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->down_right.data) {
      rosidl_runtime_c__String__init(&ros_message->down_right);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->down_right,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'down_right'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t get_serialized_size_cg_interfaces__msg__RobotSensors(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotSensors__ros_msg_type * ros_message = static_cast<const _RobotSensors__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: up
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->up.size + 1);

  // Field name: down
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->down.size + 1);

  // Field name: left
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->left.size + 1);

  // Field name: right
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->right.size + 1);

  // Field name: up_left
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->up_left.size + 1);

  // Field name: up_right
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->up_right.size + 1);

  // Field name: down_left
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->down_left.size + 1);

  // Field name: down_right
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->down_right.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t max_serialized_size_cg_interfaces__msg__RobotSensors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: up
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: down
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: left
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: right
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: up_left
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: up_right
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: down_left
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: down_right
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cg_interfaces__msg__RobotSensors;
    is_plain =
      (
      offsetof(DataType, down_right) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
bool cdr_serialize_key_cg_interfaces__msg__RobotSensors(
  const cg_interfaces__msg__RobotSensors * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: up
  {
    const rosidl_runtime_c__String * str = &ros_message->up;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: down
  {
    const rosidl_runtime_c__String * str = &ros_message->down;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: left
  {
    const rosidl_runtime_c__String * str = &ros_message->left;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: right
  {
    const rosidl_runtime_c__String * str = &ros_message->right;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: up_left
  {
    const rosidl_runtime_c__String * str = &ros_message->up_left;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: up_right
  {
    const rosidl_runtime_c__String * str = &ros_message->up_right;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: down_left
  {
    const rosidl_runtime_c__String * str = &ros_message->down_left;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: down_right
  {
    const rosidl_runtime_c__String * str = &ros_message->down_right;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t get_serialized_size_key_cg_interfaces__msg__RobotSensors(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotSensors__ros_msg_type * ros_message = static_cast<const _RobotSensors__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: up
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->up.size + 1);

  // Field name: down
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->down.size + 1);

  // Field name: left
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->left.size + 1);

  // Field name: right
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->right.size + 1);

  // Field name: up_left
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->up_left.size + 1);

  // Field name: up_right
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->up_right.size + 1);

  // Field name: down_left
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->down_left.size + 1);

  // Field name: down_right
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->down_right.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cg_interfaces
size_t max_serialized_size_key_cg_interfaces__msg__RobotSensors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: up
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: down
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: left
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: right
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: up_left
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: up_right
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: down_left
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: down_right
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cg_interfaces__msg__RobotSensors;
    is_plain =
      (
      offsetof(DataType, down_right) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _RobotSensors__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const cg_interfaces__msg__RobotSensors * ros_message = static_cast<const cg_interfaces__msg__RobotSensors *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_cg_interfaces__msg__RobotSensors(ros_message, cdr);
}

static bool _RobotSensors__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  cg_interfaces__msg__RobotSensors * ros_message = static_cast<cg_interfaces__msg__RobotSensors *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_cg_interfaces__msg__RobotSensors(cdr, ros_message);
}

static uint32_t _RobotSensors__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_cg_interfaces__msg__RobotSensors(
      untyped_ros_message, 0));
}

static size_t _RobotSensors__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_cg_interfaces__msg__RobotSensors(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RobotSensors = {
  "cg_interfaces::msg",
  "RobotSensors",
  _RobotSensors__cdr_serialize,
  _RobotSensors__cdr_deserialize,
  _RobotSensors__get_serialized_size,
  _RobotSensors__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RobotSensors__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RobotSensors,
  get_message_typesupport_handle_function,
  &cg_interfaces__msg__RobotSensors__get_type_hash,
  &cg_interfaces__msg__RobotSensors__get_type_description,
  &cg_interfaces__msg__RobotSensors__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cg_interfaces, msg, RobotSensors)() {
  return &_RobotSensors__type_support;
}

#if defined(__cplusplus)
}
#endif
