// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice

#ifndef CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_HPP_
#define CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__srv__MoveCmd_Request __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__srv__MoveCmd_Request __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveCmd_Request_
{
  using Type = MoveCmd_Request_<ContainerAllocator>;

  explicit MoveCmd_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->direction = "";
    }
  }

  explicit MoveCmd_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : direction(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->direction = "";
    }
  }

  // field types and members
  using _direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _direction_type direction;

  // setters for named parameter idiom
  Type & set__direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->direction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__srv__MoveCmd_Request
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__srv__MoveCmd_Request
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveCmd_Request_ & other) const
  {
    if (this->direction != other.direction) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveCmd_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveCmd_Request_

// alias to use template instance with default allocator
using MoveCmd_Request =
  cg_interfaces::srv::MoveCmd_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cg_interfaces


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__srv__MoveCmd_Response __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__srv__MoveCmd_Response __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveCmd_Response_
{
  using Type = MoveCmd_Response_<ContainerAllocator>;

  explicit MoveCmd_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      std::fill<typename std::array<int8_t, 2>::iterator, int8_t>(this->robot_pos.begin(), this->robot_pos.end(), 0);
      std::fill<typename std::array<int8_t, 2>::iterator, int8_t>(this->target_pos.begin(), this->target_pos.end(), 0);
    }
  }

  explicit MoveCmd_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_pos(_alloc),
    target_pos(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      std::fill<typename std::array<int8_t, 2>::iterator, int8_t>(this->robot_pos.begin(), this->robot_pos.end(), 0);
      std::fill<typename std::array<int8_t, 2>::iterator, int8_t>(this->target_pos.begin(), this->target_pos.end(), 0);
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _robot_pos_type =
    std::array<int8_t, 2>;
  _robot_pos_type robot_pos;
  using _target_pos_type =
    std::array<int8_t, 2>;
  _target_pos_type target_pos;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__robot_pos(
    const std::array<int8_t, 2> & _arg)
  {
    this->robot_pos = _arg;
    return *this;
  }
  Type & set__target_pos(
    const std::array<int8_t, 2> & _arg)
  {
    this->target_pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__srv__MoveCmd_Response
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__srv__MoveCmd_Response
    std::shared_ptr<cg_interfaces::srv::MoveCmd_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveCmd_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->robot_pos != other.robot_pos) {
      return false;
    }
    if (this->target_pos != other.target_pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveCmd_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveCmd_Response_

// alias to use template instance with default allocator
using MoveCmd_Response =
  cg_interfaces::srv::MoveCmd_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cg_interfaces

namespace cg_interfaces
{

namespace srv
{

struct MoveCmd
{
  using Request = cg_interfaces::srv::MoveCmd_Request;
  using Response = cg_interfaces::srv::MoveCmd_Response;
};

}  // namespace srv

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__SRV__DETAIL__MOVE_CMD__STRUCT_HPP_
