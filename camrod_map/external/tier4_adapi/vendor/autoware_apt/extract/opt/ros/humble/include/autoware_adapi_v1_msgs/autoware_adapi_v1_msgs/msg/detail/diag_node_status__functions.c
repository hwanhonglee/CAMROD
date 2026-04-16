// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autoware_adapi_v1_msgs:msg/DiagNodeStatus.idl
// generated code does not contain a copyright notice
#include "autoware_adapi_v1_msgs/msg/detail/diag_node_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
autoware_adapi_v1_msgs__msg__DiagNodeStatus__init(autoware_adapi_v1_msgs__msg__DiagNodeStatus * msg)
{
  if (!msg) {
    return false;
  }
  // level
  // input_level
  // latch_level
  // is_dependent
  return true;
}

void
autoware_adapi_v1_msgs__msg__DiagNodeStatus__fini(autoware_adapi_v1_msgs__msg__DiagNodeStatus * msg)
{
  if (!msg) {
    return;
  }
  // level
  // input_level
  // latch_level
  // is_dependent
}

bool
autoware_adapi_v1_msgs__msg__DiagNodeStatus__are_equal(const autoware_adapi_v1_msgs__msg__DiagNodeStatus * lhs, const autoware_adapi_v1_msgs__msg__DiagNodeStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // level
  if (lhs->level != rhs->level) {
    return false;
  }
  // input_level
  if (lhs->input_level != rhs->input_level) {
    return false;
  }
  // latch_level
  if (lhs->latch_level != rhs->latch_level) {
    return false;
  }
  // is_dependent
  if (lhs->is_dependent != rhs->is_dependent) {
    return false;
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__DiagNodeStatus__copy(
  const autoware_adapi_v1_msgs__msg__DiagNodeStatus * input,
  autoware_adapi_v1_msgs__msg__DiagNodeStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // level
  output->level = input->level;
  // input_level
  output->input_level = input->input_level;
  // latch_level
  output->latch_level = input->latch_level;
  // is_dependent
  output->is_dependent = input->is_dependent;
  return true;
}

autoware_adapi_v1_msgs__msg__DiagNodeStatus *
autoware_adapi_v1_msgs__msg__DiagNodeStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__DiagNodeStatus * msg = (autoware_adapi_v1_msgs__msg__DiagNodeStatus *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__DiagNodeStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autoware_adapi_v1_msgs__msg__DiagNodeStatus));
  bool success = autoware_adapi_v1_msgs__msg__DiagNodeStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autoware_adapi_v1_msgs__msg__DiagNodeStatus__destroy(autoware_adapi_v1_msgs__msg__DiagNodeStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autoware_adapi_v1_msgs__msg__DiagNodeStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__init(autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__DiagNodeStatus * data = NULL;

  if (size) {
    data = (autoware_adapi_v1_msgs__msg__DiagNodeStatus *)allocator.zero_allocate(size, sizeof(autoware_adapi_v1_msgs__msg__DiagNodeStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autoware_adapi_v1_msgs__msg__DiagNodeStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autoware_adapi_v1_msgs__msg__DiagNodeStatus__fini(&data[i - 1]);
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
autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__fini(autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * array)
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
      autoware_adapi_v1_msgs__msg__DiagNodeStatus__fini(&array->data[i]);
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

autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence *
autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * array = (autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__destroy(autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__are_equal(const autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * lhs, const autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__DiagNodeStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__copy(
  const autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * input,
  autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autoware_adapi_v1_msgs__msg__DiagNodeStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autoware_adapi_v1_msgs__msg__DiagNodeStatus * data =
      (autoware_adapi_v1_msgs__msg__DiagNodeStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autoware_adapi_v1_msgs__msg__DiagNodeStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autoware_adapi_v1_msgs__msg__DiagNodeStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__DiagNodeStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
