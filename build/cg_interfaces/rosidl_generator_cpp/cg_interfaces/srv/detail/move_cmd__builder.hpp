// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/move_cmd.hpp"


#ifndef CG_INTERFACES__SRV__DETAIL__MOVE_CMD__BUILDER_HPP_
#define CG_INTERFACES__SRV__DETAIL__MOVE_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cg_interfaces/srv/detail/move_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveCmd_Request_direction
{
public:
  Init_MoveCmd_Request_direction()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cg_interfaces::srv::MoveCmd_Request direction(::cg_interfaces::srv::MoveCmd_Request::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::MoveCmd_Request>()
{
  return cg_interfaces::srv::builder::Init_MoveCmd_Request_direction();
}

}  // namespace cg_interfaces


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveCmd_Response_target_pos
{
public:
  explicit Init_MoveCmd_Response_target_pos(::cg_interfaces::srv::MoveCmd_Response & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::srv::MoveCmd_Response target_pos(::cg_interfaces::srv::MoveCmd_Response::_target_pos_type arg)
  {
    msg_.target_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Response msg_;
};

class Init_MoveCmd_Response_robot_pos
{
public:
  explicit Init_MoveCmd_Response_robot_pos(::cg_interfaces::srv::MoveCmd_Response & msg)
  : msg_(msg)
  {}
  Init_MoveCmd_Response_target_pos robot_pos(::cg_interfaces::srv::MoveCmd_Response::_robot_pos_type arg)
  {
    msg_.robot_pos = std::move(arg);
    return Init_MoveCmd_Response_target_pos(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Response msg_;
};

class Init_MoveCmd_Response_success
{
public:
  Init_MoveCmd_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveCmd_Response_robot_pos success(::cg_interfaces::srv::MoveCmd_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveCmd_Response_robot_pos(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::MoveCmd_Response>()
{
  return cg_interfaces::srv::builder::Init_MoveCmd_Response_success();
}

}  // namespace cg_interfaces


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveCmd_Event_response
{
public:
  explicit Init_MoveCmd_Event_response(::cg_interfaces::srv::MoveCmd_Event & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::srv::MoveCmd_Event response(::cg_interfaces::srv::MoveCmd_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Event msg_;
};

class Init_MoveCmd_Event_request
{
public:
  explicit Init_MoveCmd_Event_request(::cg_interfaces::srv::MoveCmd_Event & msg)
  : msg_(msg)
  {}
  Init_MoveCmd_Event_response request(::cg_interfaces::srv::MoveCmd_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveCmd_Event_response(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Event msg_;
};

class Init_MoveCmd_Event_info
{
public:
  Init_MoveCmd_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveCmd_Event_request info(::cg_interfaces::srv::MoveCmd_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveCmd_Event_request(msg_);
  }

private:
  ::cg_interfaces::srv::MoveCmd_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::MoveCmd_Event>()
{
  return cg_interfaces::srv::builder::Init_MoveCmd_Event_info();
}

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__SRV__DETAIL__MOVE_CMD__BUILDER_HPP_
