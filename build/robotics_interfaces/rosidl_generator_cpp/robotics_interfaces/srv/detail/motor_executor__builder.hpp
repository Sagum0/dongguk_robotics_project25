// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotics_interfaces:srv/MotorExecutor.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__BUILDER_HPP_
#define ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotics_interfaces/srv/detail/motor_executor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotics_interfaces
{

namespace srv
{

namespace builder
{

class Init_MotorExecutor_Request_task
{
public:
  explicit Init_MotorExecutor_Request_task(::robotics_interfaces::srv::MotorExecutor_Request & msg)
  : msg_(msg)
  {}
  ::robotics_interfaces::srv::MotorExecutor_Request task(::robotics_interfaces::srv::MotorExecutor_Request::_task_type arg)
  {
    msg_.task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotics_interfaces::srv::MotorExecutor_Request msg_;
};

class Init_MotorExecutor_Request_r
{
public:
  explicit Init_MotorExecutor_Request_r(::robotics_interfaces::srv::MotorExecutor_Request & msg)
  : msg_(msg)
  {}
  Init_MotorExecutor_Request_task r(::robotics_interfaces::srv::MotorExecutor_Request::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_MotorExecutor_Request_task(msg_);
  }

private:
  ::robotics_interfaces::srv::MotorExecutor_Request msg_;
};

class Init_MotorExecutor_Request_z
{
public:
  explicit Init_MotorExecutor_Request_z(::robotics_interfaces::srv::MotorExecutor_Request & msg)
  : msg_(msg)
  {}
  Init_MotorExecutor_Request_r z(::robotics_interfaces::srv::MotorExecutor_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MotorExecutor_Request_r(msg_);
  }

private:
  ::robotics_interfaces::srv::MotorExecutor_Request msg_;
};

class Init_MotorExecutor_Request_y
{
public:
  explicit Init_MotorExecutor_Request_y(::robotics_interfaces::srv::MotorExecutor_Request & msg)
  : msg_(msg)
  {}
  Init_MotorExecutor_Request_z y(::robotics_interfaces::srv::MotorExecutor_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MotorExecutor_Request_z(msg_);
  }

private:
  ::robotics_interfaces::srv::MotorExecutor_Request msg_;
};

class Init_MotorExecutor_Request_x
{
public:
  Init_MotorExecutor_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorExecutor_Request_y x(::robotics_interfaces::srv::MotorExecutor_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MotorExecutor_Request_y(msg_);
  }

private:
  ::robotics_interfaces::srv::MotorExecutor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotics_interfaces::srv::MotorExecutor_Request>()
{
  return robotics_interfaces::srv::builder::Init_MotorExecutor_Request_x();
}

}  // namespace robotics_interfaces


namespace robotics_interfaces
{

namespace srv
{

namespace builder
{

class Init_MotorExecutor_Response_success
{
public:
  Init_MotorExecutor_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotics_interfaces::srv::MotorExecutor_Response success(::robotics_interfaces::srv::MotorExecutor_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotics_interfaces::srv::MotorExecutor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotics_interfaces::srv::MotorExecutor_Response>()
{
  return robotics_interfaces::srv::builder::Init_MotorExecutor_Response_success();
}

}  // namespace robotics_interfaces

#endif  // ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__BUILDER_HPP_
