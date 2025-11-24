// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cg_interfaces:srv/Reset.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/reset.hpp"


#ifndef CG_INTERFACES__SRV__DETAIL__RESET__BUILDER_HPP_
#define CG_INTERFACES__SRV__DETAIL__RESET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cg_interfaces/srv/detail/reset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_Reset_Request_map_name
{
public:
  explicit Init_Reset_Request_map_name(::cg_interfaces::srv::Reset_Request & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::srv::Reset_Request map_name(::cg_interfaces::srv::Reset_Request::_map_name_type arg)
  {
    msg_.map_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Request msg_;
};

class Init_Reset_Request_is_random
{
public:
  Init_Reset_Request_is_random()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Reset_Request_map_name is_random(::cg_interfaces::srv::Reset_Request::_is_random_type arg)
  {
    msg_.is_random = std::move(arg);
    return Init_Reset_Request_map_name(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::Reset_Request>()
{
  return cg_interfaces::srv::builder::Init_Reset_Request_is_random();
}

}  // namespace cg_interfaces


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_Reset_Response_loaded_map_name
{
public:
  explicit Init_Reset_Response_loaded_map_name(::cg_interfaces::srv::Reset_Response & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::srv::Reset_Response loaded_map_name(::cg_interfaces::srv::Reset_Response::_loaded_map_name_type arg)
  {
    msg_.loaded_map_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Response msg_;
};

class Init_Reset_Response_success
{
public:
  Init_Reset_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Reset_Response_loaded_map_name success(::cg_interfaces::srv::Reset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Reset_Response_loaded_map_name(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::Reset_Response>()
{
  return cg_interfaces::srv::builder::Init_Reset_Response_success();
}

}  // namespace cg_interfaces


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_Reset_Event_response
{
public:
  explicit Init_Reset_Event_response(::cg_interfaces::srv::Reset_Event & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::srv::Reset_Event response(::cg_interfaces::srv::Reset_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Event msg_;
};

class Init_Reset_Event_request
{
public:
  explicit Init_Reset_Event_request(::cg_interfaces::srv::Reset_Event & msg)
  : msg_(msg)
  {}
  Init_Reset_Event_response request(::cg_interfaces::srv::Reset_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Reset_Event_response(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Event msg_;
};

class Init_Reset_Event_info
{
public:
  Init_Reset_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Reset_Event_request info(::cg_interfaces::srv::Reset_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Reset_Event_request(msg_);
  }

private:
  ::cg_interfaces::srv::Reset_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::Reset_Event>()
{
  return cg_interfaces::srv::builder::Init_Reset_Event_info();
}

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__SRV__DETAIL__RESET__BUILDER_HPP_
