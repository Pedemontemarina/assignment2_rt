// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_message:srv/Threshold.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/threshold.hpp"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__THRESHOLD__STRUCT_HPP_
#define CUSTOM_MESSAGE__SRV__DETAIL__THRESHOLD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_message__srv__Threshold_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__srv__Threshold_Request __declspec(deprecated)
#endif

namespace custom_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Threshold_Request_
{
  using Type = Threshold_Request_<ContainerAllocator>;

  explicit Threshold_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Threshold_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    custom_message::srv::Threshold_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::srv::Threshold_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Threshold_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Threshold_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__srv__Threshold_Request
    std::shared_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__srv__Threshold_Request
    std::shared_ptr<custom_message::srv::Threshold_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Threshold_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Threshold_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Threshold_Request_

// alias to use template instance with default allocator
using Threshold_Request =
  custom_message::srv::Threshold_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_message


#ifndef _WIN32
# define DEPRECATED__custom_message__srv__Threshold_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__srv__Threshold_Response __declspec(deprecated)
#endif

namespace custom_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Threshold_Response_
{
  using Type = Threshold_Response_<ContainerAllocator>;

  explicit Threshold_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->threshold = 0.0;
    }
  }

  explicit Threshold_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->threshold = 0.0;
    }
  }

  // field types and members
  using _threshold_type =
    double;
  _threshold_type threshold;

  // setters for named parameter idiom
  Type & set__threshold(
    const double & _arg)
  {
    this->threshold = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::srv::Threshold_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::srv::Threshold_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Threshold_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Threshold_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__srv__Threshold_Response
    std::shared_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__srv__Threshold_Response
    std::shared_ptr<custom_message::srv::Threshold_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Threshold_Response_ & other) const
  {
    if (this->threshold != other.threshold) {
      return false;
    }
    return true;
  }
  bool operator!=(const Threshold_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Threshold_Response_

// alias to use template instance with default allocator
using Threshold_Response =
  custom_message::srv::Threshold_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_message


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_message__srv__Threshold_Event __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__srv__Threshold_Event __declspec(deprecated)
#endif

namespace custom_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Threshold_Event_
{
  using Type = Threshold_Event_<ContainerAllocator>;

  explicit Threshold_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Threshold_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<custom_message::srv::Threshold_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Threshold_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<custom_message::srv::Threshold_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Threshold_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<custom_message::srv::Threshold_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Threshold_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<custom_message::srv::Threshold_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_message::srv::Threshold_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::srv::Threshold_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::srv::Threshold_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Threshold_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::srv::Threshold_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__srv__Threshold_Event
    std::shared_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__srv__Threshold_Event
    std::shared_ptr<custom_message::srv::Threshold_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Threshold_Event_ & other) const
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
  bool operator!=(const Threshold_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Threshold_Event_

// alias to use template instance with default allocator
using Threshold_Event =
  custom_message::srv::Threshold_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_message

namespace custom_message
{

namespace srv
{

struct Threshold
{
  using Request = custom_message::srv::Threshold_Request;
  using Response = custom_message::srv::Threshold_Response;
  using Event = custom_message::srv::Threshold_Event;
};

}  // namespace srv

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__THRESHOLD__STRUCT_HPP_
