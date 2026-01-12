// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_message:msg/Distance.idl
// generated code does not contain a copyright notice
#include "custom_message/msg/detail/distance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `direction`
#include "rosidl_runtime_c/string_functions.h"

bool
custom_message__msg__Distance__init(custom_message__msg__Distance * msg)
{
  if (!msg) {
    return false;
  }
  // distance
  // direction
  if (!rosidl_runtime_c__String__init(&msg->direction)) {
    custom_message__msg__Distance__fini(msg);
    return false;
  }
  // threshold
  return true;
}

void
custom_message__msg__Distance__fini(custom_message__msg__Distance * msg)
{
  if (!msg) {
    return;
  }
  // distance
  // direction
  rosidl_runtime_c__String__fini(&msg->direction);
  // threshold
}

bool
custom_message__msg__Distance__are_equal(const custom_message__msg__Distance * lhs, const custom_message__msg__Distance * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  // threshold
  if (lhs->threshold != rhs->threshold) {
    return false;
  }
  return true;
}

bool
custom_message__msg__Distance__copy(
  const custom_message__msg__Distance * input,
  custom_message__msg__Distance * output)
{
  if (!input || !output) {
    return false;
  }
  // distance
  output->distance = input->distance;
  // direction
  if (!rosidl_runtime_c__String__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  // threshold
  output->threshold = input->threshold;
  return true;
}

custom_message__msg__Distance *
custom_message__msg__Distance__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_message__msg__Distance * msg = (custom_message__msg__Distance *)allocator.allocate(sizeof(custom_message__msg__Distance), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_message__msg__Distance));
  bool success = custom_message__msg__Distance__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_message__msg__Distance__destroy(custom_message__msg__Distance * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_message__msg__Distance__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_message__msg__Distance__Sequence__init(custom_message__msg__Distance__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_message__msg__Distance * data = NULL;

  if (size) {
    data = (custom_message__msg__Distance *)allocator.zero_allocate(size, sizeof(custom_message__msg__Distance), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_message__msg__Distance__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_message__msg__Distance__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_message__msg__Distance__Sequence__fini(custom_message__msg__Distance__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_message__msg__Distance__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_message__msg__Distance__Sequence *
custom_message__msg__Distance__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_message__msg__Distance__Sequence * array = (custom_message__msg__Distance__Sequence *)allocator.allocate(sizeof(custom_message__msg__Distance__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_message__msg__Distance__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_message__msg__Distance__Sequence__destroy(custom_message__msg__Distance__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_message__msg__Distance__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_message__msg__Distance__Sequence__are_equal(const custom_message__msg__Distance__Sequence * lhs, const custom_message__msg__Distance__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_message__msg__Distance__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_message__msg__Distance__Sequence__copy(
  const custom_message__msg__Distance__Sequence * input,
  custom_message__msg__Distance__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_message__msg__Distance);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_message__msg__Distance * data =
      (custom_message__msg__Distance *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_message__msg__Distance__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_message__msg__Distance__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_message__msg__Distance__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
