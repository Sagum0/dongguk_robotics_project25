// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotics_interfaces:msg/TrajectoryPoints.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__BUILDER_HPP_
#define ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotics_interfaces/msg/detail/trajectory_points__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotics_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrajectoryPoints_points
{
public:
  Init_TrajectoryPoints_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotics_interfaces::msg::TrajectoryPoints points(::robotics_interfaces::msg::TrajectoryPoints::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotics_interfaces::msg::TrajectoryPoints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotics_interfaces::msg::TrajectoryPoints>()
{
  return robotics_interfaces::msg::builder::Init_TrajectoryPoints_points();
}

}  // namespace robotics_interfaces

#endif  // ROBOTICS_INTERFACES__MSG__DETAIL__TRAJECTORY_POINTS__BUILDER_HPP_
