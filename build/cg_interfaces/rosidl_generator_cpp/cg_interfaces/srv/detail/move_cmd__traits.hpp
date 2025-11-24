// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/srv/move_cmd.hpp"


#ifndef CG_INTERFACES__SRV__DETAIL__MOVE_CMD__TRAITS_HPP_
#define CG_INTERFACES__SRV__DETAIL__MOVE_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cg_interfaces/srv/detail/move_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveCmd_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveCmd_Request & msg, bool use_flow_style = false)
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
  const cg_interfaces::srv::MoveCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::MoveCmd_Request & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::MoveCmd_Request>()
{
  return "cg_interfaces::srv::MoveCmd_Request";
}

template<>
inline const char * name<cg_interfaces::srv::MoveCmd_Request>()
{
  return "cg_interfaces/srv/MoveCmd_Request";
}

template<>
struct has_fixed_size<cg_interfaces::srv::MoveCmd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::srv::MoveCmd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cg_interfaces::srv::MoveCmd_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace cg_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveCmd_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: robot_pos
  {
    if (msg.robot_pos.size() == 0) {
      out << "robot_pos: []";
    } else {
      out << "robot_pos: [";
      size_t pending_items = msg.robot_pos.size();
      for (auto item : msg.robot_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_pos
  {
    if (msg.target_pos.size() == 0) {
      out << "target_pos: []";
    } else {
      out << "target_pos: [";
      size_t pending_items = msg.target_pos.size();
      for (auto item : msg.target_pos) {
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
  const MoveCmd_Response & msg,
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

  // member: robot_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_pos.size() == 0) {
      out << "robot_pos: []\n";
    } else {
      out << "robot_pos:\n";
      for (auto item : msg.robot_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_pos.size() == 0) {
      out << "target_pos: []\n";
    } else {
      out << "target_pos:\n";
      for (auto item : msg.target_pos) {
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

inline std::string to_yaml(const MoveCmd_Response & msg, bool use_flow_style = false)
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
  const cg_interfaces::srv::MoveCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::MoveCmd_Response & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::MoveCmd_Response>()
{
  return "cg_interfaces::srv::MoveCmd_Response";
}

template<>
inline const char * name<cg_interfaces::srv::MoveCmd_Response>()
{
  return "cg_interfaces/srv/MoveCmd_Response";
}

template<>
struct has_fixed_size<cg_interfaces::srv::MoveCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cg_interfaces::srv::MoveCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cg_interfaces::srv::MoveCmd_Response>
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
  const MoveCmd_Event & msg,
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
  const MoveCmd_Event & msg,
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

inline std::string to_yaml(const MoveCmd_Event & msg, bool use_flow_style = false)
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
  const cg_interfaces::srv::MoveCmd_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  cg_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cg_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const cg_interfaces::srv::MoveCmd_Event & msg)
{
  return cg_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<cg_interfaces::srv::MoveCmd_Event>()
{
  return "cg_interfaces::srv::MoveCmd_Event";
}

template<>
inline const char * name<cg_interfaces::srv::MoveCmd_Event>()
{
  return "cg_interfaces/srv/MoveCmd_Event";
}

template<>
struct has_fixed_size<cg_interfaces::srv::MoveCmd_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cg_interfaces::srv::MoveCmd_Event>
  : std::integral_constant<bool, has_bounded_size<cg_interfaces::srv::MoveCmd_Request>::value && has_bounded_size<cg_interfaces::srv::MoveCmd_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<cg_interfaces::srv::MoveCmd_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<cg_interfaces::srv::MoveCmd>()
{
  return "cg_interfaces::srv::MoveCmd";
}

template<>
inline const char * name<cg_interfaces::srv::MoveCmd>()
{
  return "cg_interfaces/srv/MoveCmd";
}

template<>
struct has_fixed_size<cg_interfaces::srv::MoveCmd>
  : std::integral_constant<
    bool,
    has_fixed_size<cg_interfaces::srv::MoveCmd_Request>::value &&
    has_fixed_size<cg_interfaces::srv::MoveCmd_Response>::value
  >
{
};

template<>
struct has_bounded_size<cg_interfaces::srv::MoveCmd>
  : std::integral_constant<
    bool,
    has_bounded_size<cg_interfaces::srv::MoveCmd_Request>::value &&
    has_bounded_size<cg_interfaces::srv::MoveCmd_Response>::value
  >
{
};

template<>
struct is_service<cg_interfaces::srv::MoveCmd>
  : std::true_type
{
};

template<>
struct is_service_request<cg_interfaces::srv::MoveCmd_Request>
  : std::true_type
{
};

template<>
struct is_service_response<cg_interfaces::srv::MoveCmd_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CG_INTERFACES__SRV__DETAIL__MOVE_CMD__TRAITS_HPP_
