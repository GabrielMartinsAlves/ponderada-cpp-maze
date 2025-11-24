// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice

#include "cg_interfaces/msg/detail/robot_sensors__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cg_interfaces
const rosidl_type_hash_t *
cg_interfaces__msg__RobotSensors__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0xe1, 0xdd, 0xa8, 0x90, 0x49, 0x46, 0xfc,
      0xcf, 0x53, 0x79, 0x44, 0x8c, 0x86, 0xfd, 0xb2,
      0xe8, 0x0c, 0xbc, 0xd2, 0xd0, 0x0a, 0x51, 0x0e,
      0xe4, 0xd5, 0x0f, 0x9b, 0xff, 0xf9, 0xbb, 0xf2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cg_interfaces__msg__RobotSensors__TYPE_NAME[] = "cg_interfaces/msg/RobotSensors";

// Define type names, field names, and default values
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__up[] = "up";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__down[] = "down";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__left[] = "left";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__right[] = "right";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__up_left[] = "up_left";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__up_right[] = "up_right";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__down_left[] = "down_left";
static char cg_interfaces__msg__RobotSensors__FIELD_NAME__down_right[] = "down_right";

static rosidl_runtime_c__type_description__Field cg_interfaces__msg__RobotSensors__FIELDS[] = {
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__up, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__down, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__left, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__right, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__up_left, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__up_right, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__down_left, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__msg__RobotSensors__FIELD_NAME__down_right, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cg_interfaces__msg__RobotSensors__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cg_interfaces__msg__RobotSensors__TYPE_NAME, 30, 30},
      {cg_interfaces__msg__RobotSensors__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string up\n"
  "string down\n"
  "string left\n"
  "string right\n"
  "string up_left\n"
  "string up_right\n"
  "string down_left\n"
  "string down_right";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cg_interfaces__msg__RobotSensors__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cg_interfaces__msg__RobotSensors__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 113, 113},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cg_interfaces__msg__RobotSensors__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cg_interfaces__msg__RobotSensors__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
