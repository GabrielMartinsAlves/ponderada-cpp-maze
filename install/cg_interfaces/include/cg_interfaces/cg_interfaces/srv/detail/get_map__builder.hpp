// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice

#ifndef CG_INTERFACES__SRV__DETAIL__GET_MAP__BUILDER_HPP_
#define CG_INTERFACES__SRV__DETAIL__GET_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cg_interfaces/srv/detail/get_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cg_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::GetMap_Request>()
{
  return ::cg_interfaces::srv::GetMap_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace cg_interfaces


namespace cg_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetMap_Response_occupancy_grid_shape
{
public:
  explicit Init_GetMap_Response_occupancy_grid_shape(::cg_interfaces::srv::GetMap_Response & msg)
  : msg_(msg)
  {}
  ::cg_interfaces::srv::GetMap_Response occupancy_grid_shape(::cg_interfaces::srv::GetMap_Response::_occupancy_grid_shape_type arg)
  {
    msg_.occupancy_grid_shape = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cg_interfaces::srv::GetMap_Response msg_;
};

class Init_GetMap_Response_occupancy_grid_flattened
{
public:
  Init_GetMap_Response_occupancy_grid_flattened()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetMap_Response_occupancy_grid_shape occupancy_grid_flattened(::cg_interfaces::srv::GetMap_Response::_occupancy_grid_flattened_type arg)
  {
    msg_.occupancy_grid_flattened = std::move(arg);
    return Init_GetMap_Response_occupancy_grid_shape(msg_);
  }

private:
  ::cg_interfaces::srv::GetMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cg_interfaces::srv::GetMap_Response>()
{
  return cg_interfaces::srv::builder::Init_GetMap_Response_occupancy_grid_flattened();
}

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__SRV__DETAIL__GET_MAP__BUILDER_HPP_
