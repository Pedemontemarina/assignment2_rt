// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_message:msg/Distance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/distance.hpp"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__STRUCT_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_message__msg__Distance __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__msg__Distance __declspec(deprecated)
#endif

namespace custom_message
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Distance_
{
  using Type = Distance_<ContainerAllocator>;

  explicit Distance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0f;
      this->direction = "";
      this->threshold = 0.0f;
    }
  }

  explicit Distance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : direction(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0f;
      this->direction = "";
      this->threshold = 0.0f;
    }
  }

  // field types and members
  using _distance_type =
    float;
  _distance_type distance;
  using _direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _direction_type direction;
  using _threshold_type =
    float;
  _threshold_type threshold;

  // setters for named parameter idiom
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__threshold(
    const float & _arg)
  {
    this->threshold = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::msg::Distance_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::msg::Distance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::msg::Distance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::msg::Distance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::msg::Distance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::msg::Distance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::msg::Distance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::msg::Distance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::msg::Distance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::msg::Distance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__msg__Distance
    std::shared_ptr<custom_message::msg::Distance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__msg__Distance
    std::shared_ptr<custom_message::msg::Distance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Distance_ & other) const
  {
    if (this->distance != other.distance) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->threshold != other.threshold) {
      return false;
    }
    return true;
  }
  bool operator!=(const Distance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Distance_

// alias to use template instance with default allocator
using Distance =
  custom_message::msg::Distance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__STRUCT_HPP_
