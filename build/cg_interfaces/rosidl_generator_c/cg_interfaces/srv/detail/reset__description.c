// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cg_interfaces:srv/Reset.idl
// generated code does not contain a copyright notice

#include "cg_interfaces/srv/detail/reset__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cg_interfaces
const rosidl_type_hash_t *
cg_interfaces__srv__Reset__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x79, 0xc2, 0x48, 0x2a, 0xb4, 0x2d, 0x3f, 0xf7,
      0x47, 0x61, 0x96, 0xf3, 0xbe, 0xbf, 0x47, 0xe5,
      0x5e, 0x72, 0x30, 0x69, 0xfb, 0x5e, 0x9d, 0xe8,
      0xb2, 0xe3, 0xb3, 0xd7, 0xf1, 0x59, 0x61, 0xc5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cg_interfaces
const rosidl_type_hash_t *
cg_interfaces__srv__Reset_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4a, 0xf8, 0x96, 0xb7, 0x4b, 0xe3, 0x66, 0xbe,
      0xd8, 0x5e, 0xb3, 0x3a, 0x20, 0x75, 0x9a, 0xb4,
      0x67, 0x64, 0x3d, 0x69, 0x1b, 0x1c, 0xff, 0xd4,
      0x39, 0x1e, 0xbf, 0x79, 0x1a, 0x90, 0x10, 0xa2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cg_interfaces
const rosidl_type_hash_t *
cg_interfaces__srv__Reset_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0xff, 0x06, 0xd1, 0x81, 0x47, 0x20, 0x0c,
      0x97, 0x6a, 0x32, 0x97, 0x43, 0x62, 0x2d, 0xdb,
      0xee, 0xdf, 0x72, 0x65, 0xf7, 0x1b, 0x42, 0x0f,
      0xc0, 0xa8, 0xc8, 0x12, 0x20, 0xc5, 0xd7, 0xeb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cg_interfaces
const rosidl_type_hash_t *
cg_interfaces__srv__Reset_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0x03, 0xfe, 0x49, 0x3a, 0x37, 0xd0, 0x38,
      0x36, 0xc9, 0x85, 0x7c, 0x3c, 0x46, 0xb7, 0xf6,
      0x35, 0x2b, 0xd3, 0xf2, 0x56, 0xc9, 0x1b, 0x07,
      0xf4, 0xb1, 0x91, 0x04, 0xf1, 0x56, 0xed, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char cg_interfaces__srv__Reset__TYPE_NAME[] = "cg_interfaces/srv/Reset";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char cg_interfaces__srv__Reset_Event__TYPE_NAME[] = "cg_interfaces/srv/Reset_Event";
static char cg_interfaces__srv__Reset_Request__TYPE_NAME[] = "cg_interfaces/srv/Reset_Request";
static char cg_interfaces__srv__Reset_Response__TYPE_NAME[] = "cg_interfaces/srv/Reset_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char cg_interfaces__srv__Reset__FIELD_NAME__request_message[] = "request_message";
static char cg_interfaces__srv__Reset__FIELD_NAME__response_message[] = "response_message";
static char cg_interfaces__srv__Reset__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field cg_interfaces__srv__Reset__FIELDS[] = {
  {
    {cg_interfaces__srv__Reset__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cg_interfaces__srv__Reset_Request__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cg_interfaces__srv__Reset_Response__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cg_interfaces__srv__Reset_Event__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cg_interfaces__srv__Reset__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Event__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Request__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Response__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cg_interfaces__srv__Reset__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cg_interfaces__srv__Reset__TYPE_NAME, 23, 23},
      {cg_interfaces__srv__Reset__FIELDS, 3, 3},
    },
    {cg_interfaces__srv__Reset__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = cg_interfaces__srv__Reset_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = cg_interfaces__srv__Reset_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cg_interfaces__srv__Reset_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cg_interfaces__srv__Reset_Request__FIELD_NAME__is_random[] = "is_random";
static char cg_interfaces__srv__Reset_Request__FIELD_NAME__map_name[] = "map_name";

static rosidl_runtime_c__type_description__Field cg_interfaces__srv__Reset_Request__FIELDS[] = {
  {
    {cg_interfaces__srv__Reset_Request__FIELD_NAME__is_random, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Request__FIELD_NAME__map_name, 8, 8},
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
cg_interfaces__srv__Reset_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cg_interfaces__srv__Reset_Request__TYPE_NAME, 31, 31},
      {cg_interfaces__srv__Reset_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cg_interfaces__srv__Reset_Response__FIELD_NAME__success[] = "success";
static char cg_interfaces__srv__Reset_Response__FIELD_NAME__loaded_map_name[] = "loaded_map_name";

static rosidl_runtime_c__type_description__Field cg_interfaces__srv__Reset_Response__FIELDS[] = {
  {
    {cg_interfaces__srv__Reset_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Response__FIELD_NAME__loaded_map_name, 15, 15},
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
cg_interfaces__srv__Reset_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cg_interfaces__srv__Reset_Response__TYPE_NAME, 32, 32},
      {cg_interfaces__srv__Reset_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cg_interfaces__srv__Reset_Event__FIELD_NAME__info[] = "info";
static char cg_interfaces__srv__Reset_Event__FIELD_NAME__request[] = "request";
static char cg_interfaces__srv__Reset_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field cg_interfaces__srv__Reset_Event__FIELDS[] = {
  {
    {cg_interfaces__srv__Reset_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cg_interfaces__srv__Reset_Request__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cg_interfaces__srv__Reset_Response__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cg_interfaces__srv__Reset_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Request__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {cg_interfaces__srv__Reset_Response__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cg_interfaces__srv__Reset_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cg_interfaces__srv__Reset_Event__TYPE_NAME, 29, 29},
      {cg_interfaces__srv__Reset_Event__FIELDS, 3, 3},
    },
    {cg_interfaces__srv__Reset_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = cg_interfaces__srv__Reset_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = cg_interfaces__srv__Reset_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request\n"
  "bool is_random\n"
  "string map_name  # e.g., \"1.csv\". If empty and is_random is false, resets current map.\n"
  "---\n"
  "# Response\n"
  "bool success\n"
  "string loaded_map_name";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cg_interfaces__srv__Reset__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cg_interfaces__srv__Reset__TYPE_NAME, 23, 23},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 162, 162},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cg_interfaces__srv__Reset_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cg_interfaces__srv__Reset_Request__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cg_interfaces__srv__Reset_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cg_interfaces__srv__Reset_Response__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cg_interfaces__srv__Reset_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cg_interfaces__srv__Reset_Event__TYPE_NAME, 29, 29},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cg_interfaces__srv__Reset__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cg_interfaces__srv__Reset__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cg_interfaces__srv__Reset_Event__get_individual_type_description_source(NULL);
    sources[3] = *cg_interfaces__srv__Reset_Request__get_individual_type_description_source(NULL);
    sources[4] = *cg_interfaces__srv__Reset_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cg_interfaces__srv__Reset_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cg_interfaces__srv__Reset_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cg_interfaces__srv__Reset_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cg_interfaces__srv__Reset_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cg_interfaces__srv__Reset_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cg_interfaces__srv__Reset_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cg_interfaces__srv__Reset_Request__get_individual_type_description_source(NULL);
    sources[3] = *cg_interfaces__srv__Reset_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
