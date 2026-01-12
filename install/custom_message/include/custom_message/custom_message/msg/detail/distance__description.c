// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_message:msg/Distance.idl
// generated code does not contain a copyright notice

#include "custom_message/msg/detail/distance__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__msg__Distance__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x99, 0xe2, 0xfc, 0x05, 0xff, 0x4e, 0x0a, 0x67,
      0x57, 0x95, 0xd9, 0x2c, 0x2f, 0x55, 0x23, 0x85,
      0xae, 0x70, 0x51, 0x20, 0x6c, 0xd0, 0x67, 0x74,
      0x60, 0x4b, 0x92, 0x9e, 0x8e, 0x96, 0x3c, 0xfe,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char custom_message__msg__Distance__TYPE_NAME[] = "custom_message/msg/Distance";

// Define type names, field names, and default values
static char custom_message__msg__Distance__FIELD_NAME__distance[] = "distance";
static char custom_message__msg__Distance__FIELD_NAME__direction[] = "direction";
static char custom_message__msg__Distance__FIELD_NAME__threshold[] = "threshold";

static rosidl_runtime_c__type_description__Field custom_message__msg__Distance__FIELDS[] = {
  {
    {custom_message__msg__Distance__FIELD_NAME__distance, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__msg__Distance__FIELD_NAME__direction, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_message__msg__Distance__FIELD_NAME__threshold, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_message__msg__Distance__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_message__msg__Distance__TYPE_NAME, 27, 27},
      {custom_message__msg__Distance__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Distance of the closest detected obstacle (in meters)\n"
  "float32 distance\n"
  "\n"
  "# Direction of the obstacle relative to the robot\n"
  "# Allowed values: \"left\", \"front\", \"right\"\n"
  "string direction\n"
  "\n"
  "# Safety threshold used for comparison (in meters)\n"
  "float32 threshold";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_message__msg__Distance__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_message__msg__Distance__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 254, 254},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__msg__Distance__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_message__msg__Distance__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
