// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_message:srv/FixPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/fix_point.hpp"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__FIX_POINT__BUILDER_HPP_
#define CUSTOM_MESSAGE__SRV__DETAIL__FIX_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_message/srv/detail/fix_point__struct.hpp"
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
auto build<::custom_message::srv::FixPoint_Request>()
{
  return ::custom_message::srv::FixPoint_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_message


namespace custom_message
{

namespace srv
{

namespace builder
{

class Init_FixPoint_Response_y
{
public:
  explicit Init_FixPoint_Response_y(::custom_message::srv::FixPoint_Response & msg)
  : msg_(msg)
  {}
  ::custom_message::srv::FixPoint_Response y(::custom_message::srv::FixPoint_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::srv::FixPoint_Response msg_;
};

class Init_FixPoint_Response_x
{
public:
  Init_FixPoint_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FixPoint_Response_y x(::custom_message::srv::FixPoint_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_FixPoint_Response_y(msg_);
  }

private:
  ::custom_message::srv::FixPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::FixPoint_Response>()
{
  return custom_message::srv::builder::Init_FixPoint_Response_x();
}

}  // namespace custom_message


namespace custom_message
{

namespace srv
{

namespace builder
{

class Init_FixPoint_Event_response
{
public:
  explicit Init_FixPoint_Event_response(::custom_message::srv::FixPoint_Event & msg)
  : msg_(msg)
  {}
  ::custom_message::srv::FixPoint_Event response(::custom_message::srv::FixPoint_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::srv::FixPoint_Event msg_;
};

class Init_FixPoint_Event_request
{
public:
  explicit Init_FixPoint_Event_request(::custom_message::srv::FixPoint_Event & msg)
  : msg_(msg)
  {}
  Init_FixPoint_Event_response request(::custom_message::srv::FixPoint_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_FixPoint_Event_response(msg_);
  }

private:
  ::custom_message::srv::FixPoint_Event msg_;
};

class Init_FixPoint_Event_info
{
public:
  Init_FixPoint_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FixPoint_Event_request info(::custom_message::srv::FixPoint_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_FixPoint_Event_request(msg_);
  }

private:
  ::custom_message::srv::FixPoint_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::FixPoint_Event>()
{
  return custom_message::srv::builder::Init_FixPoint_Event_info();
}

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__FIX_POINT__BUILDER_HPP_
