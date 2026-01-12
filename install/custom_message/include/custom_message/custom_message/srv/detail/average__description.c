// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_message:srv/Average.idl
// generated code does not contain a copyright notice

#include "custom_message/srv/detail/average__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__srv__Average__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4c, 0xa4, 0xf9, 0x3f, 0xc5, 0xb9, 0x43, 0xc4,
      0x8b, 0xbf, 0x56, 0x25, 0xf9, 0x78, 0x82, 0xa0,
      0x8e, 0x4c, 0x3f, 0x01, 0x94, 0x8e, 0xe5, 0xf7,
      0x4d, 0xcf, 0x2c, 0xc8, 0x74, 0x34, 0x58, 0x60,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__srv__Average_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x52, 0xe6, 0x20, 0xfb, 0x1f, 0x2b, 0xe3, 0xd2,
      0x52, 0xaf, 0x6a, 0xb3, 0x40, 0x1c, 0x26, 0x1d,
      0xe4, 0x1f, 0x03, 0xbb, 0x7e, 0xd9, 0xd4, 0xe3,
      0x61, 0x1f, 0x75, 0x61, 0x33, 0x5a, 0x4c, 0xec,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__srv__Average_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbb, 0xbf, 0x2a, 0xcb, 0xdb, 0x94, 0xcb, 0x13,
      0xe6, 0x8c, 0x7f, 0x24, 0x5a, 0xbf, 0xaf, 0x92,
      0x1e, 0xe2, 0x75, 0x38, 0xe1, 0xa2, 0xfd, 0xa1,
      0x64, 0x3d, 0xf8, 0xce, 0x21, 0x49, 0x52, 0x5b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__srv__Average_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd0, 0xd5, 0xd9, 0xc6, 0xab, 0x6f, 0xe9, 0x43,
      0xdf, 0x45, 0xd9, 0x7d, 0x2b, 0xf2, 0x61, 0xf8,
      0x0c, 0x30, 0xb7, 0xbd, 0xf9, 0x98, 0x5d, 0x43,
      0xd7, 0xe4, 0xbb, 0xc5, 0x83, 0x00, 0x72, 0x05,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char custom_message__srv__Average__TYPE_NAME[] = "custom_message/srv/Average";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char custom_message__srv__Average_Event__TYPE_NAME[] = "custom_message/srv/Average_Event";
static char custom_message__srv__Average_Request__TYPE_NAME[] = "custom_message/srv/Average_Request";
static char custom_message__srv__Average_Response__TYPE_NAME[] = "custom_message/srv/Average_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char custom_message__srv__Average__FIELD_NAME__request_message[] = "request_message";
static char custom_message__srv__Average__FIELD_NAME__response_message[] = "response_message";
static char custom_message__srv__Average__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field custom_message__srv__Average__FIELDS[] = {
  {
    {custom_message__srv__Average__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {custom_message__srv__Average_Request__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {custom_message__srv__Average_Response__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {custom_message__srv__Average_Event__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription custom_message__srv__Average__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Event__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Request__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Response__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_message__srv__Average__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_message__srv__Average__TYPE_NAME, 26, 26},
      {custom_message__srv__Average__FIELDS, 3, 3},
    },
    {custom_message__srv__Average__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = custom_message__srv__Average_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = custom_message__srv__Average_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = custom_message__srv__Average_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char custom_message__srv__Average_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field custom_message__srv__Average_Request__FIELDS[] = {
  {
    {custom_message__srv__Average_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_message__srv__Average_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_message__srv__Average_Request__TYPE_NAME, 34, 34},
      {custom_message__srv__Average_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char custom_message__srv__Average_Response__FIELD_NAME__avg_linear[] = "avg_linear";
static char custom_message__srv__Average_Response__FIELD_NAME__avg_angular[] = "avg_angular";

static rosidl_runtime_c__type_description__Field custom_message__srv__Average_Response__FIELDS[] = {
  {
    {custom_message__srv__Average_Response__FIELD_NAME__avg_linear, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Response__FIELD_NAME__avg_angular, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_message__srv__Average_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_message__srv__Average_Response__TYPE_NAME, 35, 35},
      {custom_message__srv__Average_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char custom_message__srv__Average_Event__FIELD_NAME__info[] = "info";
static char custom_message__srv__Average_Event__FIELD_NAME__request[] = "request";
static char custom_message__srv__Average_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field custom_message__srv__Average_Event__FIELDS[] = {
  {
    {custom_message__srv__Average_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {custom_message__srv__Average_Request__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {custom_message__srv__Average_Response__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription custom_message__srv__Average_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Request__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {custom_message__srv__Average_Response__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_message__srv__Average_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_message__srv__Average_Event__TYPE_NAME, 32, 32},
      {custom_message__srv__Average_Event__FIELDS, 3, 3},
    },
    {custom_message__srv__Average_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = custom_message__srv__Average_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = custom_message__srv__Average_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "float64 avg_linear \n"
  "float64 avg_angular";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_message__srv__Average__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_message__srv__Average__TYPE_NAME, 26, 26},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 43, 43},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
custom_message__srv__Average_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_message__srv__Average_Request__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
custom_message__srv__Average_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_message__srv__Average_Response__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
custom_message__srv__Average_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_message__srv__Average_Event__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__srv__Average__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_message__srv__Average__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *custom_message__srv__Average_Event__get_individual_type_description_source(NULL);
    sources[3] = *custom_message__srv__Average_Request__get_individual_type_description_source(NULL);
    sources[4] = *custom_message__srv__Average_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__srv__Average_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_message__srv__Average_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__srv__Average_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_message__srv__Average_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__srv__Average_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_message__srv__Average_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *custom_message__srv__Average_Request__get_individual_type_description_source(NULL);
    sources[3] = *custom_message__srv__Average_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
