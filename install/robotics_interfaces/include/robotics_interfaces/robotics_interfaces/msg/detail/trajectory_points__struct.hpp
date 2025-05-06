// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotics_interfaces:msg/TrajectoryPoints.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__STRUCT_HPP_
#define ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotics_interfaces__msg__TrajectoryPoints __attribute__((deprecated))
#else
# define DEPRECATED__robotics_interfaces__msg__TrajectoryPoints __declspec(deprecated)
#endif

namespace robotics_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryPoints_
{
  using Type = TrajectoryPoints_<ContainerAllocator>;

  explicit TrajectoryPoints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TrajectoryPoints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotics_interfaces__msg__TrajectoryPoints
    std::shared_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotics_interfaces__msg__TrajectoryPoints
    std::shared_ptr<robotics_interfaces::msg::TrajectoryPoints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryPoints_ & other) const
  {
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryPoints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryPoints_

// alias to use template instance with default allocator
using TrajectoryPoints =
  robotics_interfaces::msg::TrajectoryPoints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotics_interfaces

#endif  // ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__STRUCT_HPP_
