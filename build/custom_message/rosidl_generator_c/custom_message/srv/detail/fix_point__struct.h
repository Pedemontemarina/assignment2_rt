// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:srv/FixPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/fix_point.h"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__FIX_POINT__STRUCT_H_
#define CUSTOM_MESSAGE__SRV__DETAIL__FIX_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/FixPoint in the package custom_message.
typedef struct custom_message__srv__FixPoint_Request
{
  uint8_t structure_needs_at_least_one_member;
} custom_message__srv__FixPoint_Request;

// Struct for a sequence of custom_message__srv__FixPoint_Request.
typedef struct custom_message__srv__FixPoint_Request__Sequence
{
  custom_message__srv__FixPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__srv__FixPoint_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/FixPoint in the package custom_message.
typedef struct custom_message__srv__FixPoint_Response
{
  double x;
  double y;
} custom_message__srv__FixPoint_Response;

// Struct for a sequence of custom_message__srv__FixPoint_Response.
typedef struct custom_message__srv__FixPoint_Response__Sequence
{
  custom_message__srv__FixPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__srv__FixPoint_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  custom_message__srv__FixPoint_Event__request__MAX_SIZE = 1
};
// response
enum
{
  custom_message__srv__FixPoint_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/FixPoint in the package custom_message.
typedef struct custom_message__srv__FixPoint_Event
{
  service_msgs__msg__ServiceEventInfo info;
  custom_message__srv__FixPoint_Request__Sequence request;
  custom_message__srv__FixPoint_Response__Sequence response;
} custom_message__srv__FixPoint_Event;

// Struct for a sequence of custom_message__srv__FixPoint_Event.
typedef struct custom_message__srv__FixPoint_Event__Sequence
{
  custom_message__srv__FixPoint_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__srv__FixPoint_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__FIX_POINT__STRUCT_H_
