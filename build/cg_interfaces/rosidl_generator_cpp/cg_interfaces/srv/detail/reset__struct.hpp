// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cg_interfaces:srv/Reset.idl
// generated code does not contain a copyright notice

#ifndef CG_INTERFACES__SRV__DETAIL__RESET__STRUCT_HPP_
#define CG_INTERFACES__SRV__DETAIL__RESET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__srv__Reset_Request __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__srv__Reset_Request __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Reset_Request_
{
  using Type = Reset_Request_<ContainerAllocator>;

  explicit Reset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_random = false;
      this->map_name = "";
    }
  }

  explicit Reset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_random = false;
      this->map_name = "";
    }
  }

  // field types and members
  using _is_random_type =
    bool;
  _is_random_type is_random;
  using _map_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_name_type map_name;

  // setters for named parameter idiom
  Type & set__is_random(
    const bool & _arg)
  {
    this->is_random = _arg;
    return *this;
  }
  Type & set__map_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::srv::Reset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::srv::Reset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::Reset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::Reset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__srv__Reset_Request
    std::shared_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__srv__Reset_Request
    std::shared_ptr<cg_interfaces::srv::Reset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Reset_Request_ & other) const
  {
    if (this->is_random != other.is_random) {
      return false;
    }
    if (this->map_name != other.map_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Reset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Reset_Request_

// alias to use template instance with default allocator
using Reset_Request =
  cg_interfaces::srv::Reset_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cg_interfaces


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__srv__Reset_Response __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__srv__Reset_Response __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Reset_Response_
{
  using Type = Reset_Response_<ContainerAllocator>;

  explicit Reset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->loaded_map_name = "";
    }
  }

  explicit Reset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : loaded_map_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->loaded_map_name = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _loaded_map_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _loaded_map_name_type loaded_map_name;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__loaded_map_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->loaded_map_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::srv::Reset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::srv::Reset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::Reset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::Reset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__srv__Reset_Response
    std::shared_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__srv__Reset_Response
    std::shared_ptr<cg_interfaces::srv::Reset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Reset_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->loaded_map_name != other.loaded_map_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Reset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Reset_Response_

// alias to use template instance with default allocator
using Reset_Response =
  cg_interfaces::srv::Reset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cg_interfaces

namespace cg_interfaces
{

namespace srv
{

struct Reset
{
  using Request = cg_interfaces::srv::Reset_Request;
  using Response = cg_interfaces::srv::Reset_Response;
};

}  // namespace srv

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__SRV__DETAIL__RESET__STRUCT_HPP_
