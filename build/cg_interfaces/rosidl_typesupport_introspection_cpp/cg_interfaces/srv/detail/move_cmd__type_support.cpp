// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cg_interfaces/srv/detail/move_cmd__functions.h"
#include "cg_interfaces/srv/detail/move_cmd__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCmd_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cg_interfaces::srv::MoveCmd_Request(_init);
}

void MoveCmd_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cg_interfaces::srv::MoveCmd_Request *>(message_memory);
  typed_message->~MoveCmd_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCmd_Request_message_member_array[1] = {
  {
    "direction",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Request, direction),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCmd_Request_message_members = {
  "cg_interfaces::srv",  // message namespace
  "MoveCmd_Request",  // message name
  1,  // number of fields
  sizeof(cg_interfaces::srv::MoveCmd_Request),
  false,  // has_any_key_member_
  MoveCmd_Request_message_member_array,  // message members
  MoveCmd_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCmd_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCmd_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCmd_Request_message_members,
  get_message_typesupport_handle_function,
  &cg_interfaces__srv__MoveCmd_Request__get_type_hash,
  &cg_interfaces__srv__MoveCmd_Request__get_type_description,
  &cg_interfaces__srv__MoveCmd_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cg_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Request>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, MoveCmd_Request)() {
  return &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__functions.h"
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCmd_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cg_interfaces::srv::MoveCmd_Response(_init);
}

void MoveCmd_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cg_interfaces::srv::MoveCmd_Response *>(message_memory);
  typed_message->~MoveCmd_Response();
}

size_t size_function__MoveCmd_Response__robot_pos(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__MoveCmd_Response__robot_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int8_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveCmd_Response__robot_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int8_t, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveCmd_Response__robot_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__MoveCmd_Response__robot_pos(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__MoveCmd_Response__robot_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__MoveCmd_Response__robot_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

size_t size_function__MoveCmd_Response__target_pos(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__MoveCmd_Response__target_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int8_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveCmd_Response__target_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int8_t, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveCmd_Response__target_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__MoveCmd_Response__target_pos(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__MoveCmd_Response__target_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__MoveCmd_Response__target_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCmd_Response_message_member_array[3] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Response, robot_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveCmd_Response__robot_pos,  // size() function pointer
    get_const_function__MoveCmd_Response__robot_pos,  // get_const(index) function pointer
    get_function__MoveCmd_Response__robot_pos,  // get(index) function pointer
    fetch_function__MoveCmd_Response__robot_pos,  // fetch(index, &value) function pointer
    assign_function__MoveCmd_Response__robot_pos,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "target_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Response, target_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveCmd_Response__target_pos,  // size() function pointer
    get_const_function__MoveCmd_Response__target_pos,  // get_const(index) function pointer
    get_function__MoveCmd_Response__target_pos,  // get(index) function pointer
    fetch_function__MoveCmd_Response__target_pos,  // fetch(index, &value) function pointer
    assign_function__MoveCmd_Response__target_pos,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCmd_Response_message_members = {
  "cg_interfaces::srv",  // message namespace
  "MoveCmd_Response",  // message name
  3,  // number of fields
  sizeof(cg_interfaces::srv::MoveCmd_Response),
  false,  // has_any_key_member_
  MoveCmd_Response_message_member_array,  // message members
  MoveCmd_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCmd_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCmd_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCmd_Response_message_members,
  get_message_typesupport_handle_function,
  &cg_interfaces__srv__MoveCmd_Response__get_type_hash,
  &cg_interfaces__srv__MoveCmd_Response__get_type_description,
  &cg_interfaces__srv__MoveCmd_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cg_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Response>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, MoveCmd_Response)() {
  return &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__functions.h"
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCmd_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cg_interfaces::srv::MoveCmd_Event(_init);
}

void MoveCmd_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cg_interfaces::srv::MoveCmd_Event *>(message_memory);
  typed_message->~MoveCmd_Event();
}

size_t size_function__MoveCmd_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cg_interfaces::srv::MoveCmd_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MoveCmd_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cg_interfaces::srv::MoveCmd_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveCmd_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cg_interfaces::srv::MoveCmd_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveCmd_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cg_interfaces::srv::MoveCmd_Request *>(
    get_const_function__MoveCmd_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<cg_interfaces::srv::MoveCmd_Request *>(untyped_value);
  value = item;
}

void assign_function__MoveCmd_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cg_interfaces::srv::MoveCmd_Request *>(
    get_function__MoveCmd_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const cg_interfaces::srv::MoveCmd_Request *>(untyped_value);
  item = value;
}

void resize_function__MoveCmd_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cg_interfaces::srv::MoveCmd_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MoveCmd_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cg_interfaces::srv::MoveCmd_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MoveCmd_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cg_interfaces::srv::MoveCmd_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveCmd_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cg_interfaces::srv::MoveCmd_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveCmd_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cg_interfaces::srv::MoveCmd_Response *>(
    get_const_function__MoveCmd_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<cg_interfaces::srv::MoveCmd_Response *>(untyped_value);
  value = item;
}

void assign_function__MoveCmd_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cg_interfaces::srv::MoveCmd_Response *>(
    get_function__MoveCmd_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const cg_interfaces::srv::MoveCmd_Response *>(untyped_value);
  item = value;
}

void resize_function__MoveCmd_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cg_interfaces::srv::MoveCmd_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCmd_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveCmd_Event__request,  // size() function pointer
    get_const_function__MoveCmd_Event__request,  // get_const(index) function pointer
    get_function__MoveCmd_Event__request,  // get(index) function pointer
    fetch_function__MoveCmd_Event__request,  // fetch(index, &value) function pointer
    assign_function__MoveCmd_Event__request,  // assign(index, value) function pointer
    resize_function__MoveCmd_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(cg_interfaces::srv::MoveCmd_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveCmd_Event__response,  // size() function pointer
    get_const_function__MoveCmd_Event__response,  // get_const(index) function pointer
    get_function__MoveCmd_Event__response,  // get(index) function pointer
    fetch_function__MoveCmd_Event__response,  // fetch(index, &value) function pointer
    assign_function__MoveCmd_Event__response,  // assign(index, value) function pointer
    resize_function__MoveCmd_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCmd_Event_message_members = {
  "cg_interfaces::srv",  // message namespace
  "MoveCmd_Event",  // message name
  3,  // number of fields
  sizeof(cg_interfaces::srv::MoveCmd_Event),
  false,  // has_any_key_member_
  MoveCmd_Event_message_member_array,  // message members
  MoveCmd_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCmd_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCmd_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCmd_Event_message_members,
  get_message_typesupport_handle_function,
  &cg_interfaces__srv__MoveCmd_Event__get_type_hash,
  &cg_interfaces__srv__MoveCmd_Event__get_type_description,
  &cg_interfaces__srv__MoveCmd_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cg_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Event>()
{
  return &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, MoveCmd_Event)() {
  return &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__functions.h"
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace cg_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MoveCmd_service_members = {
  "cg_interfaces::srv",  // service namespace
  "MoveCmd",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<cg_interfaces::srv::MoveCmd>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t MoveCmd_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCmd_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cg_interfaces::srv::MoveCmd_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<cg_interfaces::srv::MoveCmd>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<cg_interfaces::srv::MoveCmd>,
  &cg_interfaces__srv__MoveCmd__get_type_hash,
  &cg_interfaces__srv__MoveCmd__get_type_description,
  &cg_interfaces__srv__MoveCmd__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cg_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<cg_interfaces::srv::MoveCmd>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::cg_interfaces::srv::rosidl_typesupport_introspection_cpp::MoveCmd_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cg_interfaces::srv::MoveCmd_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cg_interfaces::srv::MoveCmd_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cg_interfaces::srv::MoveCmd_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cg_interfaces, srv, MoveCmd)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<cg_interfaces::srv::MoveCmd>();
}

#ifdef __cplusplus
}
#endif
