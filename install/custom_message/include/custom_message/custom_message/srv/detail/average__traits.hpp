// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_message:srv/Average.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/average.hpp"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__TRAITS_HPP_
#define CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_message/srv/detail/average__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_message
{

namespace srv
{

inline void to_flow_style_yaml(
  const Average_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Average_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Average_Request & msg, bool use_flow_style = false)
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

}  // namespace custom_message

namespace rosidl_generator_traits
{

[[deprecated("use custom_message::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_message::srv::Average_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_message::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_message::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_message::srv::Average_Request & msg)
{
  return custom_message::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_message::srv::Average_Request>()
{
  return "custom_message::srv::Average_Request";
}

template<>
inline const char * name<custom_message::srv::Average_Request>()
{
  return "custom_message/srv/Average_Request";
}

template<>
struct has_fixed_size<custom_message::srv::Average_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_message::srv::Average_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_message::srv::Average_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_message
{

namespace srv
{

inline void to_flow_style_yaml(
  const Average_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: avg_linear
  {
    out << "avg_linear: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_linear, out);
    out << ", ";
  }

  // member: avg_angular
  {
    out << "avg_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_angular, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Average_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: avg_linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_linear: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_linear, out);
    out << "\n";
  }

  // member: avg_angular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_angular: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_angular, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Average_Response & msg, bool use_flow_style = false)
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

}  // namespace custom_message

namespace rosidl_generator_traits
{

[[deprecated("use custom_message::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_message::srv::Average_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_message::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_message::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_message::srv::Average_Response & msg)
{
  return custom_message::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_message::srv::Average_Response>()
{
  return "custom_message::srv::Average_Response";
}

template<>
inline const char * name<custom_message::srv::Average_Response>()
{
  return "custom_message/srv/Average_Response";
}

template<>
struct has_fixed_size<custom_message::srv::Average_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_message::srv::Average_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_message::srv::Average_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace custom_message
{

namespace srv
{

inline void to_flow_style_yaml(
  const Average_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Average_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Average_Event & msg, bool use_flow_style = false)
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

}  // namespace custom_message

namespace rosidl_generator_traits
{

[[deprecated("use custom_message::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_message::srv::Average_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_message::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_message::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_message::srv::Average_Event & msg)
{
  return custom_message::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_message::srv::Average_Event>()
{
  return "custom_message::srv::Average_Event";
}

template<>
inline const char * name<custom_message::srv::Average_Event>()
{
  return "custom_message/srv/Average_Event";
}

template<>
struct has_fixed_size<custom_message::srv::Average_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_message::srv::Average_Event>
  : std::integral_constant<bool, has_bounded_size<custom_message::srv::Average_Request>::value && has_bounded_size<custom_message::srv::Average_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<custom_message::srv::Average_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_message::srv::Average>()
{
  return "custom_message::srv::Average";
}

template<>
inline const char * name<custom_message::srv::Average>()
{
  return "custom_message/srv/Average";
}

template<>
struct has_fixed_size<custom_message::srv::Average>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_message::srv::Average_Request>::value &&
    has_fixed_size<custom_message::srv::Average_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_message::srv::Average>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_message::srv::Average_Request>::value &&
    has_bounded_size<custom_message::srv::Average_Response>::value
  >
{
};

template<>
struct is_service<custom_message::srv::Average>
  : std::true_type
{
};

template<>
struct is_service_request<custom_message::srv::Average_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_message::srv::Average_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__TRAITS_HPP_
