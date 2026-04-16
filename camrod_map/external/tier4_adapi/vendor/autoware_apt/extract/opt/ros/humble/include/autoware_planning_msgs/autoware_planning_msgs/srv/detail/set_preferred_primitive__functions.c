// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autoware_planning_msgs:srv/SetPreferredPrimitive.idl
// generated code does not contain a copyright notice
#include "autoware_planning_msgs/srv/detail/set_preferred_primitive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `preferred_primitives`
#include "autoware_planning_msgs/msg/detail/lanelet_primitive__functions.h"
// Member `uuid`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init(autoware_planning_msgs__srv__SetPreferredPrimitive_Request * msg)
{
  if (!msg) {
    return false;
  }
  // preferred_primitives
  if (!autoware_planning_msgs__msg__LaneletPrimitive__Sequence__init(&msg->preferred_primitives, 0)) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(msg);
    return false;
  }
  // reset
  // uuid
  if (!unique_identifier_msgs__msg__UUID__init(&msg->uuid)) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(msg);
    return false;
  }
  return true;
}

void
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(autoware_planning_msgs__srv__SetPreferredPrimitive_Request * msg)
{
  if (!msg) {
    return;
  }
  // preferred_primitives
  autoware_planning_msgs__msg__LaneletPrimitive__Sequence__fini(&msg->preferred_primitives);
  // reset
  // uuid
  unique_identifier_msgs__msg__UUID__fini(&msg->uuid);
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__are_equal(const autoware_planning_msgs__srv__SetPreferredPrimitive_Request * lhs, const autoware_planning_msgs__srv__SetPreferredPrimitive_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // preferred_primitives
  if (!autoware_planning_msgs__msg__LaneletPrimitive__Sequence__are_equal(
      &(lhs->preferred_primitives), &(rhs->preferred_primitives)))
  {
    return false;
  }
  // reset
  if (lhs->reset != rhs->reset) {
    return false;
  }
  // uuid
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->uuid), &(rhs->uuid)))
  {
    return false;
  }
  return true;
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__copy(
  const autoware_planning_msgs__srv__SetPreferredPrimitive_Request * input,
  autoware_planning_msgs__srv__SetPreferredPrimitive_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // preferred_primitives
  if (!autoware_planning_msgs__msg__LaneletPrimitive__Sequence__copy(
      &(input->preferred_primitives), &(output->preferred_primitives)))
  {
    return false;
  }
  // reset
  output->reset = input->reset;
  // uuid
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->uuid), &(output->uuid)))
  {
    return false;
  }
  return true;
}

autoware_planning_msgs__srv__SetPreferredPrimitive_Request *
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_planning_msgs__srv__SetPreferredPrimitive_Request * msg = (autoware_planning_msgs__srv__SetPreferredPrimitive_Request *)allocator.allocate(sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Request));
  bool success = autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__destroy(autoware_planning_msgs__srv__SetPreferredPrimitive_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__init(autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_planning_msgs__srv__SetPreferredPrimitive_Request * data = NULL;

  if (size) {
    data = (autoware_planning_msgs__srv__SetPreferredPrimitive_Request *)allocator.zero_allocate(size, sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(&data[i - 1]);
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
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__fini(autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * array)
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
      autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(&array->data[i]);
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

autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence *
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * array = (autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence *)allocator.allocate(sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__destroy(autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__are_equal(const autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * lhs, const autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autoware_planning_msgs__srv__SetPreferredPrimitive_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__copy(
  const autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * input,
  autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autoware_planning_msgs__srv__SetPreferredPrimitive_Request * data =
      (autoware_planning_msgs__srv__SetPreferredPrimitive_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autoware_planning_msgs__srv__SetPreferredPrimitive_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autoware_planning_msgs__srv__SetPreferredPrimitive_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `status`
#include "autoware_common_msgs/msg/detail/response_status__functions.h"

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init(autoware_planning_msgs__srv__SetPreferredPrimitive_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!autoware_common_msgs__msg__ResponseStatus__init(&msg->status)) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Response__fini(msg);
    return false;
  }
  return true;
}

void
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__fini(autoware_planning_msgs__srv__SetPreferredPrimitive_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  autoware_common_msgs__msg__ResponseStatus__fini(&msg->status);
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__are_equal(const autoware_planning_msgs__srv__SetPreferredPrimitive_Response * lhs, const autoware_planning_msgs__srv__SetPreferredPrimitive_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!autoware_common_msgs__msg__ResponseStatus__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__copy(
  const autoware_planning_msgs__srv__SetPreferredPrimitive_Response * input,
  autoware_planning_msgs__srv__SetPreferredPrimitive_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!autoware_common_msgs__msg__ResponseStatus__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

autoware_planning_msgs__srv__SetPreferredPrimitive_Response *
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_planning_msgs__srv__SetPreferredPrimitive_Response * msg = (autoware_planning_msgs__srv__SetPreferredPrimitive_Response *)allocator.allocate(sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Response));
  bool success = autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__destroy(autoware_planning_msgs__srv__SetPreferredPrimitive_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__init(autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_planning_msgs__srv__SetPreferredPrimitive_Response * data = NULL;

  if (size) {
    data = (autoware_planning_msgs__srv__SetPreferredPrimitive_Response *)allocator.zero_allocate(size, sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autoware_planning_msgs__srv__SetPreferredPrimitive_Response__fini(&data[i - 1]);
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
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__fini(autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * array)
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
      autoware_planning_msgs__srv__SetPreferredPrimitive_Response__fini(&array->data[i]);
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

autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence *
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * array = (autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence *)allocator.allocate(sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__destroy(autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__are_equal(const autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * lhs, const autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autoware_planning_msgs__srv__SetPreferredPrimitive_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__copy(
  const autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * input,
  autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autoware_planning_msgs__srv__SetPreferredPrimitive_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autoware_planning_msgs__srv__SetPreferredPrimitive_Response * data =
      (autoware_planning_msgs__srv__SetPreferredPrimitive_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autoware_planning_msgs__srv__SetPreferredPrimitive_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autoware_planning_msgs__srv__SetPreferredPrimitive_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
