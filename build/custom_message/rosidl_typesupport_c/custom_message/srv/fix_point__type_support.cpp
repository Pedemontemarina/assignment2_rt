// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from custom_message:srv/FixPoint.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "custom_message/srv/detail/fix_point__struct.h"
#include "custom_message/srv/detail/fix_point__type_support.h"
#include "custom_message/srv/detail/fix_point__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _FixPoint_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FixPoint_Request_type_support_ids_t;

static const _FixPoint_Request_type_support_ids_t _FixPoint_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FixPoint_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FixPoint_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FixPoint_Request_type_support_symbol_names_t _FixPoint_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_message, srv, FixPoint_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, FixPoint_Request)),
  }
};

typedef struct _FixPoint_Request_type_support_data_t
{
  void * data[2];
} _FixPoint_Request_type_support_data_t;

static _FixPoint_Request_type_support_data_t _FixPoint_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FixPoint_Request_message_typesupport_map = {
  2,
  "custom_message",
  &_FixPoint_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FixPoint_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FixPoint_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FixPoint_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FixPoint_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &custom_message__srv__FixPoint_Request__get_type_hash,
  &custom_message__srv__FixPoint_Request__get_type_description,
  &custom_message__srv__FixPoint_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_message

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_message, srv, FixPoint_Request)() {
  return &::custom_message::srv::rosidl_typesupport_c::FixPoint_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_message/srv/detail/fix_point__struct.h"
// already included above
// #include "custom_message/srv/detail/fix_point__type_support.h"
// already included above
// #include "custom_message/srv/detail/fix_point__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _FixPoint_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FixPoint_Response_type_support_ids_t;

static const _FixPoint_Response_type_support_ids_t _FixPoint_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FixPoint_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FixPoint_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FixPoint_Response_type_support_symbol_names_t _FixPoint_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_message, srv, FixPoint_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, FixPoint_Response)),
  }
};

typedef struct _FixPoint_Response_type_support_data_t
{
  void * data[2];
} _FixPoint_Response_type_support_data_t;

static _FixPoint_Response_type_support_data_t _FixPoint_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FixPoint_Response_message_typesupport_map = {
  2,
  "custom_message",
  &_FixPoint_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FixPoint_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FixPoint_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FixPoint_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FixPoint_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &custom_message__srv__FixPoint_Response__get_type_hash,
  &custom_message__srv__FixPoint_Response__get_type_description,
  &custom_message__srv__FixPoint_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_message

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_message, srv, FixPoint_Response)() {
  return &::custom_message::srv::rosidl_typesupport_c::FixPoint_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_message/srv/detail/fix_point__struct.h"
// already included above
// #include "custom_message/srv/detail/fix_point__type_support.h"
// already included above
// #include "custom_message/srv/detail/fix_point__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _FixPoint_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FixPoint_Event_type_support_ids_t;

static const _FixPoint_Event_type_support_ids_t _FixPoint_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FixPoint_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FixPoint_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FixPoint_Event_type_support_symbol_names_t _FixPoint_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_message, srv, FixPoint_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, FixPoint_Event)),
  }
};

typedef struct _FixPoint_Event_type_support_data_t
{
  void * data[2];
} _FixPoint_Event_type_support_data_t;

static _FixPoint_Event_type_support_data_t _FixPoint_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FixPoint_Event_message_typesupport_map = {
  2,
  "custom_message",
  &_FixPoint_Event_message_typesupport_ids.typesupport_identifier[0],
  &_FixPoint_Event_message_typesupport_symbol_names.symbol_name[0],
  &_FixPoint_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FixPoint_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FixPoint_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &custom_message__srv__FixPoint_Event__get_type_hash,
  &custom_message__srv__FixPoint_Event__get_type_description,
  &custom_message__srv__FixPoint_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_message

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_message, srv, FixPoint_Event)() {
  return &::custom_message::srv::rosidl_typesupport_c::FixPoint_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_message/srv/detail/fix_point__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _FixPoint_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FixPoint_type_support_ids_t;

static const _FixPoint_type_support_ids_t _FixPoint_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FixPoint_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FixPoint_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FixPoint_type_support_symbol_names_t _FixPoint_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_message, srv, FixPoint)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, srv, FixPoint)),
  }
};

typedef struct _FixPoint_type_support_data_t
{
  void * data[2];
} _FixPoint_type_support_data_t;

static _FixPoint_type_support_data_t _FixPoint_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FixPoint_service_typesupport_map = {
  2,
  "custom_message",
  &_FixPoint_service_typesupport_ids.typesupport_identifier[0],
  &_FixPoint_service_typesupport_symbol_names.symbol_name[0],
  &_FixPoint_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FixPoint_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FixPoint_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &FixPoint_Request_message_type_support_handle,
  &FixPoint_Response_message_type_support_handle,
  &FixPoint_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    custom_message,
    srv,
    FixPoint
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    custom_message,
    srv,
    FixPoint
  ),
  &custom_message__srv__FixPoint__get_type_hash,
  &custom_message__srv__FixPoint__get_type_description,
  &custom_message__srv__FixPoint__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace custom_message

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, custom_message, srv, FixPoint)() {
  return &::custom_message::srv::rosidl_typesupport_c::FixPoint_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
