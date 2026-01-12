// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:msg/Distance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/distance.h"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__STRUCT_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'direction'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Distance in the package custom_message.
/**
  * Distance of the closest detected obstacle (in meters)
 */
typedef struct custom_message__msg__Distance
{
  float distance;
  /// Direction of the obstacle relative to the robot
  /// Allowed values: "left", "front", "right"
  rosidl_runtime_c__String direction;
  /// Safety threshold used for comparison (in meters)
  float threshold;
} custom_message__msg__Distance;

// Struct for a sequence of custom_message__msg__Distance.
typedef struct custom_message__msg__Distance__Sequence
{
  custom_message__msg__Distance * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__msg__Distance__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__DISTANCE__STRUCT_H_
