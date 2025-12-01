// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice

#ifndef CG_INTERFACES__SRV__DETAIL__GET_MAP__STRUCT_HPP_
#define CG_INTERFACES__SRV__DETAIL__GET_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__srv__GetMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__srv__GetMap_Request __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMap_Request_
{
  using Type = GetMap_Request_<ContainerAllocator>;

  explicit GetMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::srv::GetMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::srv::GetMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::GetMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::GetMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__srv__GetMap_Request
    std::shared_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__srv__GetMap_Request
    std::shared_ptr<cg_interfaces::srv::GetMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMap_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMap_Request_

// alias to use template instance with default allocator
using GetMap_Request =
  cg_interfaces::srv::GetMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cg_interfaces


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__srv__GetMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__srv__GetMap_Response __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMap_Response_
{
  using Type = GetMap_Response_<ContainerAllocator>;

  explicit GetMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _occupancy_grid_flattened_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _occupancy_grid_flattened_type occupancy_grid_flattened;
  using _occupancy_grid_shape_type =
    rosidl_runtime_cpp::BoundedVector<uint8_t, 2, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _occupancy_grid_shape_type occupancy_grid_shape;

  // setters for named parameter idiom
  Type & set__occupancy_grid_flattened(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->occupancy_grid_flattened = _arg;
    return *this;
  }
  Type & set__occupancy_grid_shape(
    const rosidl_runtime_cpp::BoundedVector<uint8_t, 2, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->occupancy_grid_shape = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::srv::GetMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::srv::GetMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::GetMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::GetMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__srv__GetMap_Response
    std::shared_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__srv__GetMap_Response
    std::shared_ptr<cg_interfaces::srv::GetMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMap_Response_ & other) const
  {
    if (this->occupancy_grid_flattened != other.occupancy_grid_flattened) {
      return false;
    }
    if (this->occupancy_grid_shape != other.occupancy_grid_shape) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMap_Response_

// alias to use template instance with default allocator
using GetMap_Response =
  cg_interfaces::srv::GetMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cg_interfaces

namespace cg_interfaces
{

namespace srv
{

struct GetMap
{
  using Request = cg_interfaces::srv::GetMap_Request;
  using Response = cg_interfaces::srv::GetMap_Response;
};

}  // namespace srv

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__SRV__DETAIL__GET_MAP__STRUCT_HPP_
