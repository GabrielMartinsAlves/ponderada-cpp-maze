// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cg_interfaces/msg/robot_sensors.hpp"


#ifndef CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__STRUCT_HPP_
#define CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cg_interfaces__msg__RobotSensors __attribute__((deprecated))
#else
# define DEPRECATED__cg_interfaces__msg__RobotSensors __declspec(deprecated)
#endif

namespace cg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotSensors_
{
  using Type = RobotSensors_<ContainerAllocator>;

  explicit RobotSensors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->up = "";
      this->down = "";
      this->left = "";
      this->right = "";
      this->up_left = "";
      this->up_right = "";
      this->down_left = "";
      this->down_right = "";
    }
  }

  explicit RobotSensors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : up(_alloc),
    down(_alloc),
    left(_alloc),
    right(_alloc),
    up_left(_alloc),
    up_right(_alloc),
    down_left(_alloc),
    down_right(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->up = "";
      this->down = "";
      this->left = "";
      this->right = "";
      this->up_left = "";
      this->up_right = "";
      this->down_left = "";
      this->down_right = "";
    }
  }

  // field types and members
  using _up_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _up_type up;
  using _down_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _down_type down;
  using _left_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _left_type left;
  using _right_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _right_type right;
  using _up_left_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _up_left_type up_left;
  using _up_right_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _up_right_type up_right;
  using _down_left_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _down_left_type down_left;
  using _down_right_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _down_right_type down_right;

  // setters for named parameter idiom
  Type & set__up(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->up = _arg;
    return *this;
  }
  Type & set__down(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->down = _arg;
    return *this;
  }
  Type & set__left(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->right = _arg;
    return *this;
  }
  Type & set__up_left(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->up_left = _arg;
    return *this;
  }
  Type & set__up_right(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->up_right = _arg;
    return *this;
  }
  Type & set__down_left(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->down_left = _arg;
    return *this;
  }
  Type & set__down_right(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->down_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cg_interfaces::msg::RobotSensors_<ContainerAllocator> *;
  using ConstRawPtr =
    const cg_interfaces::msg::RobotSensors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::msg::RobotSensors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cg_interfaces::msg::RobotSensors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cg_interfaces__msg__RobotSensors
    std::shared_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cg_interfaces__msg__RobotSensors
    std::shared_ptr<cg_interfaces::msg::RobotSensors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotSensors_ & other) const
  {
    if (this->up != other.up) {
      return false;
    }
    if (this->down != other.down) {
      return false;
    }
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    if (this->up_left != other.up_left) {
      return false;
    }
    if (this->up_right != other.up_right) {
      return false;
    }
    if (this->down_left != other.down_left) {
      return false;
    }
    if (this->down_right != other.down_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotSensors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotSensors_

// alias to use template instance with default allocator
using RobotSensors =
  cg_interfaces::msg::RobotSensors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cg_interfaces

#endif  // CG_INTERFACES__MSG__DETAIL__ROBOT_SENSORS__STRUCT_HPP_
