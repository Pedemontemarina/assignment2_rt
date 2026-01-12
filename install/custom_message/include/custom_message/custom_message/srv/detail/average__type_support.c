// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_message:srv/Average.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_message/srv/detail/average__rosidl_typesupport_introspection_c.h"
#include "custom_message/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_message/srv/detail/average__functions.h"
#include "custom_message/srv/detail/average__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_message__srv__Average_Request__init(message_memory);
}

void custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_fini_function(void * message_memory)
{
  custom_message__srv__Average_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__srv__Average_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_members = {
  "custom_message__srv",  // message namespace
  "Average_Request",  // message name
  1,  // number of fields
  sizeof(custom_message__srv__Average_Request),
  false,  // has_any_key_member_
  custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_member_array,  // message members
  custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_type_support_handle = {
  0,
  &custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_members,
  get_message_typesupport_handle_function,
  &custom_message__srv__Average_Request__get_type_hash,
  &custom_message__srv__Average_Request__get_type_description,
  &custom_message__srv__Average_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_message
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Request)() {
  if (!custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_type_support_handle.typesupport_identifier) {
    custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_message/srv/detail/average__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_message/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_message/srv/detail/average__functions.h"
// already included above
// #include "custom_message/srv/detail/average__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_message__srv__Average_Response__init(message_memory);
}

void custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_fini_function(void * message_memory)
{
  custom_message__srv__Average_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_member_array[2] = {
  {
    "avg_linear",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__srv__Average_Response, avg_linear),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "avg_angular",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__srv__Average_Response, avg_angular),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_members = {
  "custom_message__srv",  // message namespace
  "Average_Response",  // message name
  2,  // number of fields
  sizeof(custom_message__srv__Average_Response),
  false,  // has_any_key_member_
  custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_member_array,  // message members
  custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle = {
  0,
  &custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_members,
  get_message_typesupport_handle_function,
  &custom_message__srv__Average_Response__get_type_hash,
  &custom_message__srv__Average_Response__get_type_description,
  &custom_message__srv__Average_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_message
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Response)() {
  if (!custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle.typesupport_identifier) {
    custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_message/srv/detail/average__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_message/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_message/srv/detail/average__functions.h"
// already included above
// #include "custom_message/srv/detail/average__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "custom_message/srv/average.h"
// Member `request`
// Member `response`
// already included above
// #include "custom_message/srv/detail/average__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_message__srv__Average_Event__init(message_memory);
}

void custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_fini_function(void * message_memory)
{
  custom_message__srv__Average_Event__fini(message_memory);
}

size_t custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__size_function__Average_Event__request(
  const void * untyped_member)
{
  const custom_message__srv__Average_Request__Sequence * member =
    (const custom_message__srv__Average_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_const_function__Average_Event__request(
  const void * untyped_member, size_t index)
{
  const custom_message__srv__Average_Request__Sequence * member =
    (const custom_message__srv__Average_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_function__Average_Event__request(
  void * untyped_member, size_t index)
{
  custom_message__srv__Average_Request__Sequence * member =
    (custom_message__srv__Average_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__fetch_function__Average_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const custom_message__srv__Average_Request * item =
    ((const custom_message__srv__Average_Request *)
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_const_function__Average_Event__request(untyped_member, index));
  custom_message__srv__Average_Request * value =
    (custom_message__srv__Average_Request *)(untyped_value);
  *value = *item;
}

void custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__assign_function__Average_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  custom_message__srv__Average_Request * item =
    ((custom_message__srv__Average_Request *)
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_function__Average_Event__request(untyped_member, index));
  const custom_message__srv__Average_Request * value =
    (const custom_message__srv__Average_Request *)(untyped_value);
  *item = *value;
}

bool custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__resize_function__Average_Event__request(
  void * untyped_member, size_t size)
{
  custom_message__srv__Average_Request__Sequence * member =
    (custom_message__srv__Average_Request__Sequence *)(untyped_member);
  custom_message__srv__Average_Request__Sequence__fini(member);
  return custom_message__srv__Average_Request__Sequence__init(member, size);
}

size_t custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__size_function__Average_Event__response(
  const void * untyped_member)
{
  const custom_message__srv__Average_Response__Sequence * member =
    (const custom_message__srv__Average_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_const_function__Average_Event__response(
  const void * untyped_member, size_t index)
{
  const custom_message__srv__Average_Response__Sequence * member =
    (const custom_message__srv__Average_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_function__Average_Event__response(
  void * untyped_member, size_t index)
{
  custom_message__srv__Average_Response__Sequence * member =
    (custom_message__srv__Average_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__fetch_function__Average_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const custom_message__srv__Average_Response * item =
    ((const custom_message__srv__Average_Response *)
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_const_function__Average_Event__response(untyped_member, index));
  custom_message__srv__Average_Response * value =
    (custom_message__srv__Average_Response *)(untyped_value);
  *value = *item;
}

void custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__assign_function__Average_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  custom_message__srv__Average_Response * item =
    ((custom_message__srv__Average_Response *)
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_function__Average_Event__response(untyped_member, index));
  const custom_message__srv__Average_Response * value =
    (const custom_message__srv__Average_Response *)(untyped_value);
  *item = *value;
}

bool custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__resize_function__Average_Event__response(
  void * untyped_member, size_t size)
{
  custom_message__srv__Average_Response__Sequence * member =
    (custom_message__srv__Average_Response__Sequence *)(untyped_member);
  custom_message__srv__Average_Response__Sequence__fini(member);
  return custom_message__srv__Average_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__srv__Average_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(custom_message__srv__Average_Event, request),  // bytes offset in struct
    NULL,  // default value
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__size_function__Average_Event__request,  // size() function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_const_function__Average_Event__request,  // get_const(index) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_function__Average_Event__request,  // get(index) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__fetch_function__Average_Event__request,  // fetch(index, &value) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__assign_function__Average_Event__request,  // assign(index, value) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__resize_function__Average_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(custom_message__srv__Average_Event, response),  // bytes offset in struct
    NULL,  // default value
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__size_function__Average_Event__response,  // size() function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_const_function__Average_Event__response,  // get_const(index) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__get_function__Average_Event__response,  // get(index) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__fetch_function__Average_Event__response,  // fetch(index, &value) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__assign_function__Average_Event__response,  // assign(index, value) function pointer
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__resize_function__Average_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_members = {
  "custom_message__srv",  // message namespace
  "Average_Event",  // message name
  3,  // number of fields
  sizeof(custom_message__srv__Average_Event),
  false,  // has_any_key_member_
  custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_member_array,  // message members
  custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_type_support_handle = {
  0,
  &custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_members,
  get_message_typesupport_handle_function,
  &custom_message__srv__Average_Event__get_type_hash,
  &custom_message__srv__Average_Event__get_type_description,
  &custom_message__srv__Average_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_message
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Event)() {
  custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Request)();
  custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Response)();
  if (!custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_type_support_handle.typesupport_identifier) {
    custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_message/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_message/srv/detail/average__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_members = {
  "custom_message__srv",  // service namespace
  "Average",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_Request_message_type_support_handle,
  NULL,  // response message
  // custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle
  NULL  // event_message
  // custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle
};


static rosidl_service_type_support_t custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_type_support_handle = {
  0,
  &custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_members,
  get_service_typesupport_handle_function,
  &custom_message__srv__Average_Request__rosidl_typesupport_introspection_c__Average_Request_message_type_support_handle,
  &custom_message__srv__Average_Response__rosidl_typesupport_introspection_c__Average_Response_message_type_support_handle,
  &custom_message__srv__Average_Event__rosidl_typesupport_introspection_c__Average_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    custom_message,
    srv,
    Average
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    custom_message,
    srv,
    Average
  ),
  &custom_message__srv__Average__get_type_hash,
  &custom_message__srv__Average__get_type_description,
  &custom_message__srv__Average__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_message
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average)(void) {
  if (!custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_type_support_handle.typesupport_identifier) {
    custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, Average_Event)()->data;
  }

  return &custom_message__srv__detail__average__rosidl_typesupport_introspection_c__Average_service_type_support_handle;
}
