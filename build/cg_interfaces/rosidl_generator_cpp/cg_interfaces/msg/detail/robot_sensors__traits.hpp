// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/msg/robot_sensors.hpp"


#ifndef CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__TRAITS_HPP_
#define CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cg_interfaces/msg/detail/robot_sensors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotSensors & msg,
  std::ostream & out)
{
  out << "{";
  // member: up
  {
    out << "up: ";
    rosidl_generator_traits::value_to_yaml(msg.up, out);
    out << ", ";
  }

  // member: down
  {
    out << "down: ";
    rosidl_generator_traits::value_to_yaml(msg.down, out);
    out << ", ";
  }

  // member: left
  {
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << ", ";
  }

  // member: up_left
  {
    out << "up_left: ";
    rosidl_generator_traits::value_to_yaml(msg.up_left, out);
    out << ", ";
  }

  // member: up_right
  {
    out << "up_right: ";
    rosidl_generator_traits::value_to_yaml(msg.up_right, out);
    out << ", ";
  }

  // member: down_left
  {
    out << "down_left: ";
    rosidl_generator_traits::value_to_yaml(msg.down_left, out);
    out << ", ";
  }

  // member: down_right
  {
    out << "down_right: ";
    rosidl_generator_traits::value_to_yaml(msg.down_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotSensors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "up: ";
    rosidl_generator_traits::value_to_yaml(msg.up, out);
    out << "\n";
  }

  // member: down
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "down: ";
    rosidl_generator_traits::value_to_yaml(msg.down, out);
    out << "\n";
  }

  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << "\n";
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << "\n";
  }

  // member: up_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "up_left: ";
    rosidl_generator_traits::value_to_yaml(msg.up_left, out);
    out << "\n";
  }

  // member: up_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "up_right: ";
    rosidl_generator_traits::value_to_yaml(msg.up_right, out);
    out << "\n";
  }

  // member: down_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "down_left: ";
    rosidl_generator_traits::value_to_yaml(msg.down_left, out);
    out << "\n";
  }

  // member: down_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "down_right: ";
    rosidl_generator_traits::value_to_yaml(msg.down_right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotSensors & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace cg_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use cg_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cg_interfaces::msg::RobotSensors & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::msg::RobotSensors & msg)
{
  return cg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::msg::RobotSensors>()
{
  return "cg_interfaces::msg::RobotSensors";
}

template<>
inline const char * name<cg_interfaces::msg::RobotSensors>()
{
  return "cg_interfaces/msg/RobotSensors";
}

template<>
struct has_fixed_size<cg_interfaces::msg::RobotSensors>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::msg::RobotSensors>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cg_interfaces::msg::RobotSensors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__TRAITS_HPP_
