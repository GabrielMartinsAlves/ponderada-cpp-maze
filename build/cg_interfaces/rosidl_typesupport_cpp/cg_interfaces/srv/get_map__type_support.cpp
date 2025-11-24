// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "cg_interfaces/srv/detail/get_map__functions.h"
#include "cg_interfaces/srv/detail/get_map__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetMap_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetMap_Request_type_support_ids_t;

static const _GetMap_Request_type_support_ids_t _GetMap_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetMap_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetMap_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetMap_Request_type_support_symbol_names_t _GetMap_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cg_interfaces, srv, GetMap_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, GetMap_Request)),
  }
};

typedef struct _GetMap_Request_type_support_data_t
{
  void * data[2];
} _GetMap_Request_type_support_data_t;

static _GetMap_Request_type_support_data_t _GetMap_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetMap_Request_message_typesupport_map = {
  2,
  "cg_interfaces",
  &_GetMap_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetMap_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetMap_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetMap_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetMap_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &cg_interfaces__srv__GetMap_Request__get_type_hash,
  &cg_interfaces__srv__GetMap_Request__get_type_description,
  &cg_interfaces__srv__GetMap_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace cg_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cg_interfaces::srv::GetMap_Request>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_cpp::GetMap_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, cg_interfaces, srv, GetMap_Request)() {
  return get_message_type_support_handle<cg_interfaces::srv::GetMap_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__functions.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetMap_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetMap_Response_type_support_ids_t;

static const _GetMap_Response_type_support_ids_t _GetMap_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetMap_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetMap_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetMap_Response_type_support_symbol_names_t _GetMap_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cg_interfaces, srv, GetMap_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, GetMap_Response)),
  }
};

typedef struct _GetMap_Response_type_support_data_t
{
  void * data[2];
} _GetMap_Response_type_support_data_t;

static _GetMap_Response_type_support_data_t _GetMap_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetMap_Response_message_typesupport_map = {
  2,
  "cg_interfaces",
  &_GetMap_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetMap_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetMap_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetMap_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetMap_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &cg_interfaces__srv__GetMap_Response__get_type_hash,
  &cg_interfaces__srv__GetMap_Response__get_type_description,
  &cg_interfaces__srv__GetMap_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace cg_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cg_interfaces::srv::GetMap_Response>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_cpp::GetMap_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, cg_interfaces, srv, GetMap_Response)() {
  return get_message_type_support_handle<cg_interfaces::srv::GetMap_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__functions.h"
// already included above
// #include "cg_interfaces/srv/detail/get_map__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetMap_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetMap_Event_type_support_ids_t;

static const _GetMap_Event_type_support_ids_t _GetMap_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetMap_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetMap_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetMap_Event_type_support_symbol_names_t _GetMap_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cg_interfaces, srv, GetMap_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, GetMap_Event)),
  }
};

typedef struct _GetMap_Event_type_support_data_t
{
  void * data[2];
} _GetMap_Event_type_support_data_t;

static _GetMap_Event_type_support_data_t _GetMap_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetMap_Event_message_typesupport_map = {
  2,
  "cg_interfaces",
  &_GetMap_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetMap_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetMap_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetMap_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetMap_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &cg_interfaces__srv__GetMap_Event__get_type_hash,
  &cg_interfaces__srv__GetMap_Event__get_type_description,
  &cg_interfaces__srv__GetMap_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace cg_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cg_interfaces::srv::GetMap_Event>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_cpp::GetMap_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, cg_interfaces, srv, GetMap_Event)() {
  return get_message_type_support_handle<cg_interfaces::srv::GetMap_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "cg_interfaces/srv/detail/get_map__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetMap_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetMap_type_support_ids_t;

static const _GetMap_type_support_ids_t _GetMap_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetMap_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetMap_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetMap_type_support_symbol_names_t _GetMap_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cg_interfaces, srv, GetMap)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, GetMap)),
  }
};

typedef struct _GetMap_type_support_data_t
{
  void * data[2];
} _GetMap_type_support_data_t;

static _GetMap_type_support_data_t _GetMap_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetMap_service_typesupport_map = {
  2,
  "cg_interfaces",
  &_GetMap_service_typesupport_ids.typesupport_identifier[0],
  &_GetMap_service_typesupport_symbol_names.symbol_name[0],
  &_GetMap_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetMap_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetMap_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<cg_interfaces::srv::GetMap_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<cg_interfaces::srv::GetMap_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<cg_interfaces::srv::GetMap_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<cg_interfaces::srv::GetMap>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<cg_interfaces::srv::GetMap>,
  &cg_interfaces__srv__GetMap__get_type_hash,
  &cg_interfaces__srv__GetMap__get_type_description,
  &cg_interfaces__srv__GetMap__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace cg_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<cg_interfaces::srv::GetMap>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_cpp::GetMap_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, cg_interfaces, srv, GetMap)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<cg_interfaces::srv::GetMap>();
}

#ifdef __cplusplus
}
#endif
