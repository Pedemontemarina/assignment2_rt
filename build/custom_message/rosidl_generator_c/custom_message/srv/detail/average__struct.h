// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:srv/Average.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/srv/average.h"


#ifndef CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__STRUCT_H_
#define CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Average in the package custom_message.
typedef struct custom_message__srv__Average_Request
{
  uint8_t structure_needs_at_least_one_member;
} custom_message__srv__Average_Request;

// Struct for a sequence of custom_message__srv__Average_Request.
typedef struct custom_message__srv__Average_Request__Sequence
{
  custom_message__srv__Average_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__srv__Average_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Average in the package custom_message.
typedef struct custom_message__srv__Average_Response
{
  double avg_linear;
  double avg_angular;
} custom_message__srv__Average_Response;

// Struct for a sequence of custom_message__srv__Average_Response.
typedef struct custom_message__srv__Average_Response__Sequence
{
  custom_message__srv__Average_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__srv__Average_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  custom_message__srv__Average_Event__request__MAX_SIZE = 1
};
// response
enum
{
  custom_message__srv__Average_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Average in the package custom_message.
typedef struct custom_message__srv__Average_Event
{
  service_msgs__msg__ServiceEventInfo info;
  custom_message__srv__Average_Request__Sequence request;
  custom_message__srv__Average_Response__Sequence response;
} custom_message__srv__Average_Event;

// Struct for a sequence of custom_message__srv__Average_Event.
typedef struct custom_message__srv__Average_Event__Sequence
{
  custom_message__srv__Average_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__srv__Average_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__SRV__DETAIL__AVERAGE__STRUCT_H_
