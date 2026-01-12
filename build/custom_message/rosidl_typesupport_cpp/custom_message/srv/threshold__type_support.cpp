// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from custom_message:srv/Threshold.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "custom_message/srv/detail/threshold__functions.h"
#include "custom_message/srv/detail/threshold__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Threshold_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Threshold_Request_type_support_ids_t;

static const _Threshold_Request_type_support_ids_t _Threshold_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Threshold_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Threshold_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Threshold_Request_type_support_symbol_names_t _Threshold_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_message, srv, Threshold_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_message, srv, Threshold_Request)),
  }
};

typedef struct _Threshold_Request_type_support_data_t
{
  void * data[2];
} _Threshold_Request_type_support_data_t;

static _Threshold_Request_type_support_data_t _Threshold_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Threshold_Request_message_typesupport_map = {
  2,
  "custom_message",
  &_Threshold_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Threshold_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Threshold_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Threshold_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Threshold_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &custom_message__srv__Threshold_Request__get_type_hash,
  &custom_message__srv__Threshold_Request__get_type_description,
  &custom_message__srv__Threshold_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace custom_message

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_message::srv::Threshold_Request>()
{
  return &::custom_message::srv::rosidl_typesupport_cpp::Threshold_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_message, srv, Threshold_Request)() {
  return get_message_type_support_handle<custom_message::srv::Threshold_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_message/srv/detail/threshold__functions.h"
// already included above
// #include "custom_message/srv/detail/threshold__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Threshold_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Threshold_Response_type_support_ids_t;

static const _Threshold_Response_type_support_ids_t _Threshold_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Threshold_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Threshold_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Threshold_Response_type_support_symbol_names_t _Threshold_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_message, srv, Threshold_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_message, srv, Threshold_Response)),
  }
};

typedef struct _Threshold_Response_type_support_data_t
{
  void * data[2];
} _Threshold_Response_type_support_data_t;

static _Threshold_Response_type_support_data_t _Threshold_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Threshold_Response_message_typesupport_map = {
  2,
  "custom_message",
  &_Threshold_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Threshold_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Threshold_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Threshold_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Threshold_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &custom_message__srv__Threshold_Response__get_type_hash,
  &custom_message__srv__Threshold_Response__get_type_description,
  &custom_message__srv__Threshold_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace custom_message

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_message::srv::Threshold_Response>()
{
  return &::custom_message::srv::rosidl_typesupport_cpp::Threshold_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_message, srv, Threshold_Response)() {
  return get_message_type_support_handle<custom_message::srv::Threshold_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_message/srv/detail/threshold__functions.h"
// already included above
// #include "custom_message/srv/detail/threshold__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Threshold_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Threshold_Event_type_support_ids_t;

static const _Threshold_Event_type_support_ids_t _Threshold_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Threshold_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Threshold_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Threshold_Event_type_support_symbol_names_t _Threshold_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_message, srv, Threshold_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_message, srv, Threshold_Event)),
  }
};

typedef struct _Threshold_Event_type_support_data_t
{
  void * data[2];
} _Threshold_Event_type_support_data_t;

static _Threshold_Event_type_support_data_t _Threshold_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Threshold_Event_message_typesupport_map = {
  2,
  "custom_message",
  &_Threshold_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Threshold_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Threshold_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Threshold_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Threshold_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &custom_message__srv__Threshold_Event__get_type_hash,
  &custom_message__srv__Threshold_Event__get_type_description,
  &custom_message__srv__Threshold_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace custom_message

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_message::srv::Threshold_Event>()
{
  return &::custom_message::srv::rosidl_typesupport_cpp::Threshold_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_message, srv, Threshold_Event)() {
  return get_message_type_support_handle<custom_message::srv::Threshold_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "custom_message/srv/detail/threshold__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_message
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Threshold_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Threshold_type_support_ids_t;

static const _Threshold_type_support_ids_t _Threshold_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Threshold_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Threshold_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Threshold_type_support_symbol_names_t _Threshold_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_message, srv, Threshold)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_message, srv, Threshold)),
  }
};

typedef struct _Threshold_type_support_data_t
{
  void * data[2];
} _Threshold_type_support_data_t;

static _Threshold_type_support_data_t _Threshold_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Threshold_service_typesupport_map = {
  2,
  "custom_message",
  &_Threshold_service_typesupport_ids.typesupport_identifier[0],
  &_Threshold_service_typesupport_symbol_names.symbol_name[0],
  &_Threshold_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Threshold_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Threshold_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<custom_message::srv::Threshold_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<custom_message::srv::Threshold_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<custom_message::srv::Threshold_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<custom_message::srv::Threshold>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<custom_message::srv::Threshold>,
  &custom_message__srv__Threshold__get_type_hash,
  &custom_message__srv__Threshold__get_type_description,
  &custom_message__srv__Threshold__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace custom_message

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<custom_message::srv::Threshold>()
{
  return &::custom_message::srv::rosidl_typesupport_cpp::Threshold_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_message, srv, Threshold)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<custom_message::srv::Threshold>();
}

#ifdef __cplusplus
}
#endif
