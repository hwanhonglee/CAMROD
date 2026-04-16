// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autoware_adapi_v1_msgs:msg/SteeringCommand.idl
// generated code does not contain a copyright notice
#include "autoware_adapi_v1_msgs/msg/detail/steering_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
autoware_adapi_v1_msgs__msg__SteeringCommand__init(autoware_adapi_v1_msgs__msg__SteeringCommand * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    autoware_adapi_v1_msgs__msg__SteeringCommand__fini(msg);
    return false;
  }
  // steering_tire_angle
  // steering_tire_velocity
  return true;
}

void
autoware_adapi_v1_msgs__msg__SteeringCommand__fini(autoware_adapi_v1_msgs__msg__SteeringCommand * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // steering_tire_angle
  // steering_tire_velocity
}

bool
autoware_adapi_v1_msgs__msg__SteeringCommand__are_equal(const autoware_adapi_v1_msgs__msg__SteeringCommand * lhs, const autoware_adapi_v1_msgs__msg__SteeringCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // steering_tire_angle
  if (lhs->steering_tire_angle != rhs->steering_tire_angle) {
    return false;
  }
  // steering_tire_velocity
  if (lhs->steering_tire_velocity != rhs->steering_tire_velocity) {
    return false;
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__SteeringCommand__copy(
  const autoware_adapi_v1_msgs__msg__SteeringCommand * input,
  autoware_adapi_v1_msgs__msg__SteeringCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // steering_tire_angle
  output->steering_tire_angle = input->steering_tire_angle;
  // steering_tire_velocity
  output->steering_tire_velocity = input->steering_tire_velocity;
  return true;
}

autoware_adapi_v1_msgs__msg__SteeringCommand *
autoware_adapi_v1_msgs__msg__SteeringCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__SteeringCommand * msg = (autoware_adapi_v1_msgs__msg__SteeringCommand *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__SteeringCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autoware_adapi_v1_msgs__msg__SteeringCommand));
  bool success = autoware_adapi_v1_msgs__msg__SteeringCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autoware_adapi_v1_msgs__msg__SteeringCommand__destroy(autoware_adapi_v1_msgs__msg__SteeringCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autoware_adapi_v1_msgs__msg__SteeringCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__init(autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__SteeringCommand * data = NULL;

  if (size) {
    data = (autoware_adapi_v1_msgs__msg__SteeringCommand *)allocator.zero_allocate(size, sizeof(autoware_adapi_v1_msgs__msg__SteeringCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autoware_adapi_v1_msgs__msg__SteeringCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autoware_adapi_v1_msgs__msg__SteeringCommand__fini(&data[i - 1]);
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
autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__fini(autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * array)
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
      autoware_adapi_v1_msgs__msg__SteeringCommand__fini(&array->data[i]);
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

autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence *
autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * array = (autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence *)allocator.allocate(sizeof(autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__destroy(autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__are_equal(const autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * lhs, const autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__SteeringCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__copy(
  const autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * input,
  autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autoware_adapi_v1_msgs__msg__SteeringCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autoware_adapi_v1_msgs__msg__SteeringCommand * data =
      (autoware_adapi_v1_msgs__msg__SteeringCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autoware_adapi_v1_msgs__msg__SteeringCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autoware_adapi_v1_msgs__msg__SteeringCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autoware_adapi_v1_msgs__msg__SteeringCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
