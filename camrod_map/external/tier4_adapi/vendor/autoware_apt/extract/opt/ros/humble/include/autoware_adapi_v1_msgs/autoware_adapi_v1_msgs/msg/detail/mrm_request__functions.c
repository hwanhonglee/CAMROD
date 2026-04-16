// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autoware_adapi_v1_msgs:msg/MrmRequest.idl
// generated code does not contain a copyright notice
#include "autoware_adapi_v1_msgs/msg/detail/mrm_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sender`
#include "rosidl_runtime_c/string_functions.h"

bool
autoware_adapi_v1_msgs__msg__MrmRequest__init(autoware_adapi_v1_msgs__msg__MrmRequest * msg)
{
  if (!msg) {
    return false;
  }
  // sender
  if (!rosidl_runtime_c__String__init(&msg->sender)) {
    autoware_adapi_v1_msgs__msg__MrmRequest__fini(msg);
    return false;
  }
  // strategy
  return true;
}

void
autoware_adapi_v1_msgs__msg__MrmRequest__fini(autoware_adapi_v1_msgs__msg__MrmRequest * msg)
{
  if (!msg) {
    return;
  }
  // sender
  rosidl_runtime_c__String__fini(&msg->sender);
  // strategy
}

bool
autoware_adapi_v1_msgs__msg__MrmRequest__are_equal(const autoware_adapi_v1_msgs__msg__MrmRequest * lhs, const autoware_adapi_v1_msgs__msg__MrmRequest * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sender
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sender), &(rhs->sender)))
  {
    return false;
  }
  // strategy
  if (lhs->strategy != rhs->strategy) {
    return false;
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__MrmRequest__copy(
  const autoware_adapi_v1_msgs__msg__MrmRequest * input,
  autoware_adapi_v1_msgs__msg__MrmRequest * output)
{
  if (!input || !output) {
    return false;
  }
  // sender
  if (!rosidl_runtime_c__String__copy(
      &(input->sender), &(output->sender)))
  {
    return false;
  }
  // strategy
  output->strategy = input->strategy;
  return true;
}

autoware_adapi_v1_msgs__msg__MrmRequest *
autoware_adapi_v1_msgs__msg__MrmRequest__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__MrmRequest * msg = (autoware_adapi_v1_msgs__msg__MrmRequest *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__MrmRequest), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autoware_adapi_v1_msgs__msg__MrmRequest));
  bool success = autoware_adapi_v1_msgs__msg__MrmRequest__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autoware_adapi_v1_msgs__msg__MrmRequest__destroy(autoware_adapi_v1_msgs__msg__MrmRequest * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autoware_adapi_v1_msgs__msg__MrmRequest__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__init(autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__MrmRequest * data = NULL;

  if (size) {
    data = (autoware_adapi_v1_msgs__msg__MrmRequest *)allocator.zero_allocate(size, sizeof(autoware_adapi_v1_msgs__msg__MrmRequest), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autoware_adapi_v1_msgs__msg__MrmRequest__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autoware_adapi_v1_msgs__msg__MrmRequest__fini(&data[i - 1]);
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
autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__fini(autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * array)
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
      autoware_adapi_v1_msgs__msg__MrmRequest__fini(&array->data[i]);
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

autoware_adapi_v1_msgs__msg__MrmRequest__Sequence *
autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * array = (autoware_adapi_v1_msgs__msg__MrmRequest__Sequence *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__MrmRequest__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__destroy(autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__are_equal(const autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * lhs, const autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__MrmRequest__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__copy(
  const autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * input,
  autoware_adapi_v1_msgs__msg__MrmRequest__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autoware_adapi_v1_msgs__msg__MrmRequest);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autoware_adapi_v1_msgs__msg__MrmRequest * data =
      (autoware_adapi_v1_msgs__msg__MrmRequest *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autoware_adapi_v1_msgs__msg__MrmRequest__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autoware_adapi_v1_msgs__msg__MrmRequest__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__MrmRequest__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
