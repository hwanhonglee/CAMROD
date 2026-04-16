// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autoware_adapi_v1_msgs:msg/MrmDescription.idl
// generated code does not contain a copyright notice
#include "autoware_adapi_v1_msgs/msg/detail/mrm_description__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

bool
autoware_adapi_v1_msgs__msg__MrmDescription__init(autoware_adapi_v1_msgs__msg__MrmDescription * msg)
{
  if (!msg) {
    return false;
  }
  // behavior
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    autoware_adapi_v1_msgs__msg__MrmDescription__fini(msg);
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__init(&msg->description)) {
    autoware_adapi_v1_msgs__msg__MrmDescription__fini(msg);
    return false;
  }
  return true;
}

void
autoware_adapi_v1_msgs__msg__MrmDescription__fini(autoware_adapi_v1_msgs__msg__MrmDescription * msg)
{
  if (!msg) {
    return;
  }
  // behavior
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // description
  rosidl_runtime_c__String__fini(&msg->description);
}

bool
autoware_adapi_v1_msgs__msg__MrmDescription__are_equal(const autoware_adapi_v1_msgs__msg__MrmDescription * lhs, const autoware_adapi_v1_msgs__msg__MrmDescription * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // behavior
  if (lhs->behavior != rhs->behavior) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->description), &(rhs->description)))
  {
    return false;
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__MrmDescription__copy(
  const autoware_adapi_v1_msgs__msg__MrmDescription * input,
  autoware_adapi_v1_msgs__msg__MrmDescription * output)
{
  if (!input || !output) {
    return false;
  }
  // behavior
  output->behavior = input->behavior;
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__copy(
      &(input->description), &(output->description)))
  {
    return false;
  }
  return true;
}

autoware_adapi_v1_msgs__msg__MrmDescription *
autoware_adapi_v1_msgs__msg__MrmDescription__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__MrmDescription * msg = (autoware_adapi_v1_msgs__msg__MrmDescription *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__MrmDescription), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autoware_adapi_v1_msgs__msg__MrmDescription));
  bool success = autoware_adapi_v1_msgs__msg__MrmDescription__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autoware_adapi_v1_msgs__msg__MrmDescription__destroy(autoware_adapi_v1_msgs__msg__MrmDescription * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autoware_adapi_v1_msgs__msg__MrmDescription__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__init(autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__MrmDescription * data = NULL;

  if (size) {
    data = (autoware_adapi_v1_msgs__msg__MrmDescription *)allocator.zero_allocate(size, sizeof(autoware_adapi_v1_msgs__msg__MrmDescription), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autoware_adapi_v1_msgs__msg__MrmDescription__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autoware_adapi_v1_msgs__msg__MrmDescription__fini(&data[i - 1]);
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
autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__fini(autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * array)
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
      autoware_adapi_v1_msgs__msg__MrmDescription__fini(&array->data[i]);
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

autoware_adapi_v1_msgs__msg__MrmDescription__Sequence *
autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * array = (autoware_adapi_v1_msgs__msg__MrmDescription__Sequence *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__MrmDescription__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__destroy(autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__are_equal(const autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * lhs, const autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__MrmDescription__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__copy(
  const autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * input,
  autoware_adapi_v1_msgs__msg__MrmDescription__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autoware_adapi_v1_msgs__msg__MrmDescription);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autoware_adapi_v1_msgs__msg__MrmDescription * data =
      (autoware_adapi_v1_msgs__msg__MrmDescription *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autoware_adapi_v1_msgs__msg__MrmDescription__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autoware_adapi_v1_msgs__msg__MrmDescription__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__MrmDescription__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
