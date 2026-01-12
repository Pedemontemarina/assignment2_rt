// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_message:srv/Threshold.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/threshold.hpp"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__THRESHOLD__BUILDER_HPP_
#define CUSTOM_MESSAGE__SRV__DETAIL__THRESHOLD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_message/srv/detail/threshold__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_message
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::Threshold_Request>()
{
  return ::custom_message::srv::Threshold_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_message


namespace custom_message
{

namespace srv
{

namespace builder
{

class Init_Threshold_Response_threshold
{
public:
  Init_Threshold_Response_threshold()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_message::srv::Threshold_Response threshold(::custom_message::srv::Threshold_Response::_threshold_type arg)
  {
    msg_.threshold = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::srv::Threshold_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::Threshold_Response>()
{
  return custom_message::srv::builder::Init_Threshold_Response_threshold();
}

}  // namespace custom_message


namespace custom_message
{

namespace srv
{

namespace builder
{

class Init_Threshold_Event_response
{
public:
  explicit Init_Threshold_Event_response(::custom_message::srv::Threshold_Event & msg)
  : msg_(msg)
  {}
  ::custom_message::srv::Threshold_Event response(::custom_message::srv::Threshold_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::srv::Threshold_Event msg_;
};

class Init_Threshold_Event_request
{
public:
  explicit Init_Threshold_Event_request(::custom_message::srv::Threshold_Event & msg)
  : msg_(msg)
  {}
  Init_Threshold_Event_response request(::custom_message::srv::Threshold_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Threshold_Event_response(msg_);
  }

private:
  ::custom_message::srv::Threshold_Event msg_;
};

class Init_Threshold_Event_info
{
public:
  Init_Threshold_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Threshold_Event_request info(::custom_message::srv::Threshold_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Threshold_Event_request(msg_);
  }

private:
  ::custom_message::srv::Threshold_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::Threshold_Event>()
{
  return custom_message::srv::builder::Init_Threshold_Event_info();
}

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__THRESHOLD__BUILDER_HPP_
