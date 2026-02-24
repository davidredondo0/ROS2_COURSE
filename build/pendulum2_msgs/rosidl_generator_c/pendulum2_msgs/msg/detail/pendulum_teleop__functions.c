// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pendulum2_msgs:msg/PendulumTeleop.idl
// generated code does not contain a copyright notice
#include "pendulum2_msgs/msg/detail/pendulum_teleop__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
pendulum2_msgs__msg__PendulumTeleop__init(pendulum2_msgs__msg__PendulumTeleop * msg)
{
  if (!msg) {
    return false;
  }
  // pole_angle
  // pole_velocity
  // cart_position
  // cart_velocity
  return true;
}

void
pendulum2_msgs__msg__PendulumTeleop__fini(pendulum2_msgs__msg__PendulumTeleop * msg)
{
  if (!msg) {
    return;
  }
  // pole_angle
  // pole_velocity
  // cart_position
  // cart_velocity
}

bool
pendulum2_msgs__msg__PendulumTeleop__are_equal(const pendulum2_msgs__msg__PendulumTeleop * lhs, const pendulum2_msgs__msg__PendulumTeleop * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pole_angle
  if (lhs->pole_angle != rhs->pole_angle) {
    return false;
  }
  // pole_velocity
  if (lhs->pole_velocity != rhs->pole_velocity) {
    return false;
  }
  // cart_position
  if (lhs->cart_position != rhs->cart_position) {
    return false;
  }
  // cart_velocity
  if (lhs->cart_velocity != rhs->cart_velocity) {
    return false;
  }
  return true;
}

bool
pendulum2_msgs__msg__PendulumTeleop__copy(
  const pendulum2_msgs__msg__PendulumTeleop * input,
  pendulum2_msgs__msg__PendulumTeleop * output)
{
  if (!input || !output) {
    return false;
  }
  // pole_angle
  output->pole_angle = input->pole_angle;
  // pole_velocity
  output->pole_velocity = input->pole_velocity;
  // cart_position
  output->cart_position = input->cart_position;
  // cart_velocity
  output->cart_velocity = input->cart_velocity;
  return true;
}

pendulum2_msgs__msg__PendulumTeleop *
pendulum2_msgs__msg__PendulumTeleop__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pendulum2_msgs__msg__PendulumTeleop * msg = (pendulum2_msgs__msg__PendulumTeleop *)allocator.allocate(sizeof(pendulum2_msgs__msg__PendulumTeleop), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pendulum2_msgs__msg__PendulumTeleop));
  bool success = pendulum2_msgs__msg__PendulumTeleop__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pendulum2_msgs__msg__PendulumTeleop__destroy(pendulum2_msgs__msg__PendulumTeleop * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pendulum2_msgs__msg__PendulumTeleop__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pendulum2_msgs__msg__PendulumTeleop__Sequence__init(pendulum2_msgs__msg__PendulumTeleop__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pendulum2_msgs__msg__PendulumTeleop * data = NULL;

  if (size) {
    data = (pendulum2_msgs__msg__PendulumTeleop *)allocator.zero_allocate(size, sizeof(pendulum2_msgs__msg__PendulumTeleop), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pendulum2_msgs__msg__PendulumTeleop__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pendulum2_msgs__msg__PendulumTeleop__fini(&data[i - 1]);
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
pendulum2_msgs__msg__PendulumTeleop__Sequence__fini(pendulum2_msgs__msg__PendulumTeleop__Sequence * array)
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
      pendulum2_msgs__msg__PendulumTeleop__fini(&array->data[i]);
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

pendulum2_msgs__msg__PendulumTeleop__Sequence *
pendulum2_msgs__msg__PendulumTeleop__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pendulum2_msgs__msg__PendulumTeleop__Sequence * array = (pendulum2_msgs__msg__PendulumTeleop__Sequence *)allocator.allocate(sizeof(pendulum2_msgs__msg__PendulumTeleop__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pendulum2_msgs__msg__PendulumTeleop__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pendulum2_msgs__msg__PendulumTeleop__Sequence__destroy(pendulum2_msgs__msg__PendulumTeleop__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pendulum2_msgs__msg__PendulumTeleop__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pendulum2_msgs__msg__PendulumTeleop__Sequence__are_equal(const pendulum2_msgs__msg__PendulumTeleop__Sequence * lhs, const pendulum2_msgs__msg__PendulumTeleop__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pendulum2_msgs__msg__PendulumTeleop__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pendulum2_msgs__msg__PendulumTeleop__Sequence__copy(
  const pendulum2_msgs__msg__PendulumTeleop__Sequence * input,
  pendulum2_msgs__msg__PendulumTeleop__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pendulum2_msgs__msg__PendulumTeleop);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pendulum2_msgs__msg__PendulumTeleop * data =
      (pendulum2_msgs__msg__PendulumTeleop *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pendulum2_msgs__msg__PendulumTeleop__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pendulum2_msgs__msg__PendulumTeleop__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pendulum2_msgs__msg__PendulumTeleop__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
