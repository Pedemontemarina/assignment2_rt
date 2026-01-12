// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_message:msg/Distance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/distance.hpp"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__BUILDER_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_message/msg/detail/distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_message
{

namespace msg
{

namespace builder
{

class Init_Distance_threshold
{
public:
  explicit Init_Distance_threshold(::custom_message::msg::Distance & msg)
  : msg_(msg)
  {}
  ::custom_message::msg::Distance threshold(::custom_message::msg::Distance::_threshold_type arg)
  {
    msg_.threshold = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::msg::Distance msg_;
};

class Init_Distance_direction
{
public:
  explicit Init_Distance_direction(::custom_message::msg::Distance & msg)
  : msg_(msg)
  {}
  Init_Distance_threshold direction(::custom_message::msg::Distance::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_Distance_threshold(msg_);
  }

private:
  ::custom_message::msg::Distance msg_;
};

class Init_Distance_distance
{
public:
  Init_Distance_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Distance_direction distance(::custom_message::msg::Distance::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_Distance_direction(msg_);
  }

private:
  ::custom_message::msg::Distance msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::msg::Distance>()
{
  return custom_message::msg::builder::Init_Distance_distance();
}

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__BUILDER_HPP_
