// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotics_interfaces:srv/MotorExecutor.idl
// generated code does not contain a copyright notice

#ifndef ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__TRAITS_HPP_
#define ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotics_interfaces/srv/detail/motor_executor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotics_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotorExecutor_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: grab
  {
    out << "grab: ";
    rosidl_generator_traits::value_to_yaml(msg.grab, out);
    out << ", ";
  }

  // member: task
  {
    out << "task: ";
    rosidl_generator_traits::value_to_yaml(msg.task, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorExecutor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: grab
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grab: ";
    rosidl_generator_traits::value_to_yaml(msg.grab, out);
    out << "\n";
  }

  // member: task
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task: ";
    rosidl_generator_traits::value_to_yaml(msg.task, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorExecutor_Request & msg, bool use_flow_style = false)
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

}  // namespace robotics_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robotics_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotics_interfaces::srv::MotorExecutor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotics_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotics_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotics_interfaces::srv::MotorExecutor_Request & msg)
{
  return robotics_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotics_interfaces::srv::MotorExecutor_Request>()
{
  return "robotics_interfaces::srv::MotorExecutor_Request";
}

template<>
inline const char * name<robotics_interfaces::srv::MotorExecutor_Request>()
{
  return "robotics_interfaces/srv/MotorExecutor_Request";
}

template<>
struct has_fixed_size<robotics_interfaces::srv::MotorExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotics_interfaces::srv::MotorExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotics_interfaces::srv::MotorExecutor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robotics_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotorExecutor_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorExecutor_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorExecutor_Response & msg, bool use_flow_style = false)
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

}  // namespace robotics_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robotics_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotics_interfaces::srv::MotorExecutor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotics_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotics_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotics_interfaces::srv::MotorExecutor_Response & msg)
{
  return robotics_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotics_interfaces::srv::MotorExecutor_Response>()
{
  return "robotics_interfaces::srv::MotorExecutor_Response";
}

template<>
inline const char * name<robotics_interfaces::srv::MotorExecutor_Response>()
{
  return "robotics_interfaces/srv/MotorExecutor_Response";
}

template<>
struct has_fixed_size<robotics_interfaces::srv::MotorExecutor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robotics_interfaces::srv::MotorExecutor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robotics_interfaces::srv::MotorExecutor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotics_interfaces::srv::MotorExecutor>()
{
  return "robotics_interfaces::srv::MotorExecutor";
}

template<>
inline const char * name<robotics_interfaces::srv::MotorExecutor>()
{
  return "robotics_interfaces/srv/MotorExecutor";
}

template<>
struct has_fixed_size<robotics_interfaces::srv::MotorExecutor>
  : std::integral_constant<
    bool,
    has_fixed_size<robotics_interfaces::srv::MotorExecutor_Request>::value &&
    has_fixed_size<robotics_interfaces::srv::MotorExecutor_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotics_interfaces::srv::MotorExecutor>
  : std::integral_constant<
    bool,
    has_bounded_size<robotics_interfaces::srv::MotorExecutor_Request>::value &&
    has_bounded_size<robotics_interfaces::srv::MotorExecutor_Response>::value
  >
{
};

template<>
struct is_service<robotics_interfaces::srv::MotorExecutor>
  : std::true_type
{
};

template<>
struct is_service_request<robotics_interfaces::srv::MotorExecutor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotics_interfaces::srv::MotorExecutor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTICS_INTERFACES__SRV__DETAIL__MOTOR_EXECUTOR__TRAITS_HPP_
