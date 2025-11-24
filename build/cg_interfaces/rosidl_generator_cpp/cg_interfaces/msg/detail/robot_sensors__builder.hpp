// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/msg/robot_sensors.hpp"


#ifndef CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__BUILDER_HPP_
#define CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cg_interfaces/msg/detail/robot_sensors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cg_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotSensors_down_right
{
public:
  explicit Init_RobotSensors_down_right(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::msg::RobotSensors down_right(::cg_interfaces::msg::RobotSensors::_down_right_type arg)
  {
    msg_.down_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_down_left
{
public:
  explicit Init_RobotSensors_down_left(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  Init_RobotSensors_down_right down_left(::cg_interfaces::msg::RobotSensors::_down_left_type arg)
  {
    msg_.down_left = std::move(arg);
    return Init_RobotSensors_down_right(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_up_right
{
public:
  explicit Init_RobotSensors_up_right(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  Init_RobotSensors_down_left up_right(::cg_interfaces::msg::RobotSensors::_up_right_type arg)
  {
    msg_.up_right = std::move(arg);
    return Init_RobotSensors_down_left(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_up_left
{
public:
  explicit Init_RobotSensors_up_left(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  Init_RobotSensors_up_right up_left(::cg_interfaces::msg::RobotSensors::_up_left_type arg)
  {
    msg_.up_left = std::move(arg);
    return Init_RobotSensors_up_right(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_right
{
public:
  explicit Init_RobotSensors_right(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  Init_RobotSensors_up_left right(::cg_interfaces::msg::RobotSensors::_right_type arg)
  {
    msg_.right = std::move(arg);
    return Init_RobotSensors_up_left(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_left
{
public:
  explicit Init_RobotSensors_left(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  Init_RobotSensors_right left(::cg_interfaces::msg::RobotSensors::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_RobotSensors_right(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_down
{
public:
  explicit Init_RobotSensors_down(::cg_interfaces::msg::RobotSensors & msg)
  : msg_(msg)
  {}
  Init_RobotSensors_left down(::cg_interfaces::msg::RobotSensors::_down_type arg)
  {
    msg_.down = std::move(arg);
    return Init_RobotSensors_left(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

class Init_RobotSensors_up
{
public:
  Init_RobotSensors_up()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotSensors_down up(::cg_interfaces::msg::RobotSensors::_up_type arg)
  {
    msg_.up = std::move(arg);
    return Init_RobotSensors_down(msg_);
  }

private:
  ::cg_interfaces::msg::RobotSensors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::msg::RobotSensors>()
{
  return cg_interfaces::msg::builder::Init_RobotSensors_up();
}

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__BUILDER_HPP_
