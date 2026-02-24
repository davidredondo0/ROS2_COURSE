// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pendulum2_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice
#include "pendulum2_msgs/msg/detail/joint_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
pendulum2_msgs__msg__JointCommand__init(pendulum2_msgs__msg__JointCommand * msg)
{
  if (!msg) {
    return false;
  }
  // force
  return true;
}

void
pendulum2_msgs__msg__JointCommand__fini(pendulum2_msgs__msg__JointCommand * msg)
{
  if (!msg) {
    return;
  }
  // force
}

bool
pendulum2_msgs__msg__JointCommand__are_equal(const pendulum2_msgs__msg__JointCommand * lhs, const pendulum2_msgs__msg__JointCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  return true;
}

bool
pendulum2_msgs__msg__JointCommand__copy(
  const pendulum2_msgs__msg__JointCommand * input,
  pendulum2_msgs__msg__JointCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // force
  output->force = input->force;
  return true;
}

pendulum2_msgs__msg__JointCommand *
pendulum2_msgs__msg__JointCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pendulum2_msgs__msg__JointCommand * msg = (pendulum2_msgs__msg__JointCommand *)allocator.allocate(sizeof(pendulum2_msgs__msg__JointCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pendulum2_msgs__msg__JointCommand));
  bool success = pendulum2_msgs__msg__JointCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pendulum2_msgs__msg__JointCommand__destroy(pendulum2_msgs__msg__JointCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pendulum2_msgs__msg__JointCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pendulum2_msgs__msg__JointCommand__Sequence__init(pendulum2_msgs__msg__JointCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pendulum2_msgs__msg__JointCommand * data = NULL;

  if (size) {
    data = (pendulum2_msgs__msg__JointCommand *)allocator.zero_allocate(size, sizeof(pendulum2_msgs__msg__JointCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pendulum2_msgs__msg__JointCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pendulum2_msgs__msg__JointCommand__fini(&data[i - 1]);
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
pendulum2_msgs__msg__JointCommand__Sequence__fini(pendulum2_msgs__msg__JointCommand__Sequence * array)
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
      pendulum2_msgs__msg__JointCommand__fini(&array->data[i]);
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

pendulum2_msgs__msg__JointCommand__Sequence *
pendulum2_msgs__msg__JointCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pendulum2_msgs__msg__JointCommand__Sequence * array = (pendulum2_msgs__msg__JointCommand__Sequence *)allocator.allocate(sizeof(pendulum2_msgs__msg__JointCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pendulum2_msgs__msg__JointCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pendulum2_msgs__msg__JointCommand__Sequence__destroy(pendulum2_msgs__msg__JointCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pendulum2_msgs__msg__JointCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pendulum2_msgs__msg__JointCommand__Sequence__are_equal(const pendulum2_msgs__msg__JointCommand__Sequence * lhs, const pendulum2_msgs__msg__JointCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pendulum2_msgs__msg__JointCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pendulum2_msgs__msg__JointCommand__Sequence__copy(
  const pendulum2_msgs__msg__JointCommand__Sequence * input,
  pendulum2_msgs__msg__JointCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pendulum2_msgs__msg__JointCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pendulum2_msgs__msg__JointCommand * data =
      (pendulum2_msgs__msg__JointCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pendulum2_msgs__msg__JointCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pendulum2_msgs__msg__JointCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pendulum2_msgs__msg__JointCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
