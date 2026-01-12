// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_message:srv/Average.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/average.hpp"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__STRUCT_HPP_
#define CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_message__srv__Average_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__srv__Average_Request __declspec(deprecated)
#endif

namespace custom_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Average_Request_
{
  using Type = Average_Request_<ContainerAllocator>;

  explicit Average_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Average_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::srv::Average_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::srv::Average_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::srv::Average_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::srv::Average_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Average_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Average_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Average_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Average_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::srv::Average_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::srv::Average_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__srv__Average_Request
    std::shared_ptr<custom_message::srv::Average_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__srv__Average_Request
    std::shared_ptr<custom_message::srv::Average_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Average_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Average_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Average_Request_

// alias to use template instance with default allocator
using Average_Request =
  custom_message::srv::Average_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_message


#ifndef _WIN32
# define DEPRECATED__custom_message__srv__Average_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__srv__Average_Response __declspec(deprecated)
#endif

namespace custom_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Average_Response_
{
  using Type = Average_Response_<ContainerAllocator>;

  explicit Average_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->avg_linear = 0.0;
      this->avg_angular = 0.0;
    }
  }

  explicit Average_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->avg_linear = 0.0;
      this->avg_angular = 0.0;
    }
  }

  // field types and members
  using _avg_linear_type =
    double;
  _avg_linear_type avg_linear;
  using _avg_angular_type =
    double;
  _avg_angular_type avg_angular;

  // setters for named parameter idiom
  Type & set__avg_linear(
    const double & _arg)
  {
    this->avg_linear = _arg;
    return *this;
  }
  Type & set__avg_angular(
    const double & _arg)
  {
    this->avg_angular = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::srv::Average_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::srv::Average_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::srv::Average_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::srv::Average_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Average_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Average_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Average_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Average_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::srv::Average_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::srv::Average_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__srv__Average_Response
    std::shared_ptr<custom_message::srv::Average_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__srv__Average_Response
    std::shared_ptr<custom_message::srv::Average_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Average_Response_ & other) const
  {
    if (this->avg_linear != other.avg_linear) {
      return false;
    }
    if (this->avg_angular != other.avg_angular) {
      return false;
    }
    return true;
  }
  bool operator!=(const Average_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Average_Response_

// alias to use template instance with default allocator
using Average_Response =
  custom_message::srv::Average_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_message


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_message__srv__Average_Event __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__srv__Average_Event __declspec(deprecated)
#endif

namespace custom_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Average_Event_
{
  using Type = Average_Event_<ContainerAllocator>;

  explicit Average_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Average_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<custom_message::srv::Average_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Average_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<custom_message::srv::Average_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Average_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<custom_message::srv::Average_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Average_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<custom_message::srv::Average_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Average_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::srv::Average_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::srv::Average_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::srv::Average_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::srv::Average_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Average_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Average_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Average_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Average_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::srv::Average_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::srv::Average_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__srv__Average_Event
    std::shared_ptr<custom_message::srv::Average_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__srv__Average_Event
    std::shared_ptr<custom_message::srv::Average_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Average_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Average_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Average_Event_

// alias to use template instance with default allocator
using Average_Event =
  custom_message::srv::Average_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_message

namespace custom_message
{

namespace srv
{

struct Average
{
  using Request = custom_message::srv::Average_Request;
  using Response = custom_message::srv::Average_Response;
  using Event = custom_message::srv::Average_Event;
};

}  // namespace srv

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__STRUCT_HPP_
