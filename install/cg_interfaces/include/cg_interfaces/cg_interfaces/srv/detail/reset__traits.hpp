// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cg_interfaces:srv/Reset.idl
// generated code does not contain a copyright notice

#ifndef CG_INTERFACES__SRV__DETAIL__RESET__TRAITS_HPP_
#define CG_INTERFACES__SRV__DETAIL__RESET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cg_interfaces/srv/detail/reset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Reset_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_random
  {
    out << "is_random: ";
    rosidl_generator_traits::value_to_yaml(msg.is_random, out);
    out << ", ";
  }

  // member: map_name
  {
    out << "map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.map_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Reset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_random
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_random: ";
    rosidl_generator_traits::value_to_yaml(msg.is_random, out);
    out << "\n";
  }

  // member: map_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.map_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Reset_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace cg_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use cg_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cg_interfaces::srv::Reset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::Reset_Request & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::Reset_Request>()
{
  return "cg_interfaces::srv::Reset_Request";
}

template<>
inline const char * name<cg_interfaces::srv::Reset_Request>()
{
  return "cg_interfaces/srv/Reset_Request";
}

template<>
struct has_fixed_size<cg_interfaces::srv::Reset_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::srv::Reset_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cg_interfaces::srv::Reset_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Reset_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: loaded_map_name
  {
    out << "loaded_map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.loaded_map_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Reset_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: loaded_map_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loaded_map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.loaded_map_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Reset_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace cg_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use cg_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cg_interfaces::srv::Reset_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::Reset_Response & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::Reset_Response>()
{
  return "cg_interfaces::srv::Reset_Response";
}

template<>
inline const char * name<cg_interfaces::srv::Reset_Response>()
{
  return "cg_interfaces/srv/Reset_Response";
}

template<>
struct has_fixed_size<cg_interfaces::srv::Reset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::srv::Reset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cg_interfaces::srv::Reset_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cg_interfaces::srv::Reset>()
{
  return "cg_interfaces::srv::Reset";
}

template<>
inline const char * name<cg_interfaces::srv::Reset>()
{
  return "cg_interfaces/srv/Reset";
}

template<>
struct has_fixed_size<cg_interfaces::srv::Reset>
  : std::integral_constant<
    bool,
    has_fixed_size<cg_interfaces::srv::Reset_Request>::value &&
    has_fixed_size<cg_interfaces::srv::Reset_Response>::value
  >
{
};

template<>
struct has_bounded_size<cg_interfaces::srv::Reset>
  : std::integral_constant<
    bool,
    has_bounded_size<cg_interfaces::srv::Reset_Request>::value &&
    has_bounded_size<cg_interfaces::srv::Reset_Response>::value
  >
{
};

template<>
struct is_service<cg_interfaces::srv::Reset>
  : std::true_type
{
};

template<>
struct is_service_request<cg_interfaces::srv::Reset_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cg_interfaces::srv::Reset_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CG_INTERFACES__SRV__DETAIL__RESET__TRAITS_HPP_
