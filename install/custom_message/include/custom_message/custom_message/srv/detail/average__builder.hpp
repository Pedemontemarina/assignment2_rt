// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_message:srv/Average.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/average.hpp"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__BUILDER_HPP_
#define CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_message/srv/detail/average__struct.hpp"
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
auto build<::custom_message::srv::Average_Request>()
{
  return ::custom_message::srv::Average_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_message


namespace custom_message
{

namespace srv
{

namespace builder
{

class Init_Average_Response_avg_angular
{
public:
  explicit Init_Average_Response_avg_angular(::custom_message::srv::Average_Response & msg)
  : msg_(msg)
  {}
  ::custom_message::srv::Average_Response avg_angular(::custom_message::srv::Average_Response::_avg_angular_type arg)
  {
    msg_.avg_angular = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::srv::Average_Response msg_;
};

class Init_Average_Response_avg_linear
{
public:
  Init_Average_Response_avg_linear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Average_Response_avg_angular avg_linear(::custom_message::srv::Average_Response::_avg_linear_type arg)
  {
    msg_.avg_linear = std::move(arg);
    return Init_Average_Response_avg_angular(msg_);
  }

private:
  ::custom_message::srv::Average_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::Average_Response>()
{
  return custom_message::srv::builder::Init_Average_Response_avg_linear();
}

}  // namespace custom_message


namespace custom_message
{

namespace srv
{

namespace builder
{

class Init_Average_Event_response
{
public:
  explicit Init_Average_Event_response(::custom_message::srv::Average_Event & msg)
  : msg_(msg)
  {}
  ::custom_message::srv::Average_Event response(::custom_message::srv::Average_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::srv::Average_Event msg_;
};

class Init_Average_Event_request
{
public:
  explicit Init_Average_Event_request(::custom_message::srv::Average_Event & msg)
  : msg_(msg)
  {}
  Init_Average_Event_response request(::custom_message::srv::Average_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Average_Event_response(msg_);
  }

private:
  ::custom_message::srv::Average_Event msg_;
};

class Init_Average_Event_info
{
public:
  Init_Average_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Average_Event_request info(::custom_message::srv::Average_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Average_Event_request(msg_);
  }

private:
  ::custom_message::srv::Average_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::srv::Average_Event>()
{
  return custom_message::srv::builder::Init_Average_Event_info();
}

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__BUILDER_HPP_
