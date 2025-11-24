// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/get_map.hpp"


#ifndef CG_INTERFACES__SRV__DETAIL__GET_MAP__TRAITS_HPP_
#define CG_INTERFACES__SRV__DETAIL__GET_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cg_interfaces/srv/detail/get_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMap_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMap_Request & msg, bool use_flow_style = false)
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
  const cg_interfaces::srv::GetMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::GetMap_Request & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::GetMap_Request>()
{
  return "cg_interfaces::srv::GetMap_Request";
}

template<>
inline const char * name<cg_interfaces::srv::GetMap_Request>()
{
  return "cg_interfaces/srv/GetMap_Request";
}

template<>
struct has_fixed_size<cg_interfaces::srv::GetMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cg_interfaces::srv::GetMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cg_interfaces::srv::GetMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: occupancy_grid_flattened
  {
    if (msg.occupancy_grid_flattened.size() == 0) {
      out << "occupancy_grid_flattened: []";
    } else {
      out << "occupancy_grid_flattened: [";
      size_t pending_items = msg.occupancy_grid_flattened.size();
      for (auto item : msg.occupancy_grid_flattened) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: occupancy_grid_shape
  {
    if (msg.occupancy_grid_shape.size() == 0) {
      out << "occupancy_grid_shape: []";
    } else {
      out << "occupancy_grid_shape: [";
      size_t pending_items = msg.occupancy_grid_shape.size();
      for (auto item : msg.occupancy_grid_shape) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: occupancy_grid_flattened
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.occupancy_grid_flattened.size() == 0) {
      out << "occupancy_grid_flattened: []\n";
    } else {
      out << "occupancy_grid_flattened:\n";
      for (auto item : msg.occupancy_grid_flattened) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: occupancy_grid_shape
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.occupancy_grid_shape.size() == 0) {
      out << "occupancy_grid_shape: []\n";
    } else {
      out << "occupancy_grid_shape:\n";
      for (auto item : msg.occupancy_grid_shape) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMap_Response & msg, bool use_flow_style = false)
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
  const cg_interfaces::srv::GetMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::GetMap_Response & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::GetMap_Response>()
{
  return "cg_interfaces::srv::GetMap_Response";
}

template<>
inline const char * name<cg_interfaces::srv::GetMap_Response>()
{
  return "cg_interfaces/srv/GetMap_Response";
}

template<>
struct has_fixed_size<cg_interfaces::srv::GetMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::srv::GetMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cg_interfaces::srv::GetMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMap_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMap_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMap_Event & msg, bool use_flow_style = false)
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
  const cg_interfaces::srv::GetMap_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::GetMap_Event & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::GetMap_Event>()
{
  return "cg_interfaces::srv::GetMap_Event";
}

template<>
inline const char * name<cg_interfaces::srv::GetMap_Event>()
{
  return "cg_interfaces/srv/GetMap_Event";
}

template<>
struct has_fixed_size<cg_interfaces::srv::GetMap_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::srv::GetMap_Event>
  : std::integral_constant<bool, has_bounded_size<cg_interfaces::srv::GetMap_Request>::value && has_bounded_size<cg_interfaces::srv::GetMap_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<cg_interfaces::srv::GetMap_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cg_interfaces::srv::GetMap>()
{
  return "cg_interfaces::srv::GetMap";
}

template<>
inline const char * name<cg_interfaces::srv::GetMap>()
{
  return "cg_interfaces/srv/GetMap";
}

template<>
struct has_fixed_size<cg_interfaces::srv::GetMap>
  : std::integral_constant<
    bool,
    has_fixed_size<cg_interfaces::srv::GetMap_Request>::value &&
    has_fixed_size<cg_interfaces::srv::GetMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<cg_interfaces::srv::GetMap>
  : std::integral_constant<
    bool,
    has_bounded_size<cg_interfaces::srv::GetMap_Request>::value &&
    has_bounded_size<cg_interfaces::srv::GetMap_Response>::value
  >
{
};

template<>
struct is_service<cg_interfaces::srv::GetMap>
  : std::true_type
{
};

template<>
struct is_service_request<cg_interfaces::srv::GetMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cg_interfaces::srv::GetMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CG_INTERFACES__SRV__DETAIL__GET_MAP__TRAITS_HPP_
