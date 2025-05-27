// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotics_interfaces:srv/MotorExecutor.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__STRUCT_HPP_
#define ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotics_interfaces__srv__MotorExecutor_Request __attribute__((deprecated))
#else
# define DEPRECATED__robotics_interfaces__srv__MotorExecutor_Request __declspec(deprecated)
#endif

namespace robotics_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorExecutor_Request_
{
  using Type = MotorExecutor_Request_<ContainerAllocator>;

  explicit MotorExecutor_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->r = 0.0f;
      this->grab = false;
      this->task = "";
      this->time = 0.0f;
    }
  }

  explicit MotorExecutor_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->r = 0.0f;
      this->grab = false;
      this->task = "";
      this->time = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _r_type =
    float;
  _r_type r;
  using _grab_type =
    bool;
  _grab_type grab;
  using _task_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_type task;
  using _time_type =
    float;
  _time_type time;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__grab(
    const bool & _arg)
  {
    this->grab = _arg;
    return *this;
  }
  Type & set__task(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task = _arg;
    return *this;
  }
  Type & set__time(
    const float & _arg)
  {
    this->time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotics_interfaces__srv__MotorExecutor_Request
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotics_interfaces__srv__MotorExecutor_Request
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorExecutor_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->grab != other.grab) {
      return false;
    }
    if (this->task != other.task) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorExecutor_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorExecutor_Request_

// alias to use template instance with default allocator
using MotorExecutor_Request =
  robotics_interfaces::srv::MotorExecutor_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotics_interfaces


#ifndef _WIN32
# define DEPRECATED__robotics_interfaces__srv__MotorExecutor_Response __attribute__((deprecated))
#else
# define DEPRECATED__robotics_interfaces__srv__MotorExecutor_Response __declspec(deprecated)
#endif

namespace robotics_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorExecutor_Response_
{
  using Type = MotorExecutor_Response_<ContainerAllocator>;

  explicit MotorExecutor_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MotorExecutor_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotics_interfaces__srv__MotorExecutor_Response
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotics_interfaces__srv__MotorExecutor_Response
    std::shared_ptr<robotics_interfaces::srv::MotorExecutor_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorExecutor_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorExecutor_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorExecutor_Response_

// alias to use template instance with default allocator
using MotorExecutor_Response =
  robotics_interfaces::srv::MotorExecutor_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotics_interfaces

namespace robotics_interfaces
{

namespace srv
{

struct MotorExecutor
{
  using Request = robotics_interfaces::srv::MotorExecutor_Request;
  using Response = robotics_interfaces::srv::MotorExecutor_Response;
};

}  // namespace srv

}  // namespace robotics_interfaces

#endif  // ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__STRUCT_HPP_
