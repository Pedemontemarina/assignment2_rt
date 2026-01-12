// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_message:msg/Distance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/distance.hpp"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__TRAITS_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_message/msg/detail/distance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_message
{

namespace msg
{

inline void to_flow_style_yaml(
  const Distance & msg,
  std::ostream & out)
{
  out << "{";
  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << ", ";
  }

  // member: threshold
  {
    out << "threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.threshold, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Distance & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }

  // member: threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.threshold, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Distance & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_message

namespace rosidl_generator_traits
{

[[deprecated("use custom_message::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_message::msg::Distance & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_message::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_message::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_message::msg::Distance & msg)
{
  return custom_message::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_message::msg::Distance>()
{
  return "custom_message::msg::Distance";
}

template<>
inline const char * name<custom_message::msg::Distance>()
{
  return "custom_message/msg/Distance";
}

template<>
struct has_fixed_size<custom_message::msg::Distance>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_message::msg::Distance>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_message::msg::Distance>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__TRAITS_HPP_
