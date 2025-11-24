// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cg_interfaces:msg/RobotSensors.idl
// generated code does not contain a copyright notice
#include "cg_interfaces/msg/detail/robot_sensors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `up`
// Member `down`
// Member `left`
// Member `right`
// Member `up_left`
// Member `up_right`
// Member `down_left`
// Member `down_right`
#include "rosidl_runtime_c/string_functions.h"

bool
cg_interfaces__msg__RobotSensors__init(cg_interfaces__msg__RobotSensors * msg)
{
  if (!msg) {
    return false;
  }
  // up
  if (!rosidl_runtime_c__String__init(&msg->up)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // down
  if (!rosidl_runtime_c__String__init(&msg->down)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // left
  if (!rosidl_runtime_c__String__init(&msg->left)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // right
  if (!rosidl_runtime_c__String__init(&msg->right)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // up_left
  if (!rosidl_runtime_c__String__init(&msg->up_left)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // up_right
  if (!rosidl_runtime_c__String__init(&msg->up_right)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // down_left
  if (!rosidl_runtime_c__String__init(&msg->down_left)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  // down_right
  if (!rosidl_runtime_c__String__init(&msg->down_right)) {
    cg_interfaces__msg__RobotSensors__fini(msg);
    return false;
  }
  return true;
}

void
cg_interfaces__msg__RobotSensors__fini(cg_interfaces__msg__RobotSensors * msg)
{
  if (!msg) {
    return;
  }
  // up
  rosidl_runtime_c__String__fini(&msg->up);
  // down
  rosidl_runtime_c__String__fini(&msg->down);
  // left
  rosidl_runtime_c__String__fini(&msg->left);
  // right
  rosidl_runtime_c__String__fini(&msg->right);
  // up_left
  rosidl_runtime_c__String__fini(&msg->up_left);
  // up_right
  rosidl_runtime_c__String__fini(&msg->up_right);
  // down_left
  rosidl_runtime_c__String__fini(&msg->down_left);
  // down_right
  rosidl_runtime_c__String__fini(&msg->down_right);
}

bool
cg_interfaces__msg__RobotSensors__are_equal(const cg_interfaces__msg__RobotSensors * lhs, const cg_interfaces__msg__RobotSensors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // up
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->up), &(rhs->up)))
  {
    return false;
  }
  // down
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->down), &(rhs->down)))
  {
    return false;
  }
  // left
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->left), &(rhs->left)))
  {
    return false;
  }
  // right
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->right), &(rhs->right)))
  {
    return false;
  }
  // up_left
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->up_left), &(rhs->up_left)))
  {
    return false;
  }
  // up_right
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->up_right), &(rhs->up_right)))
  {
    return false;
  }
  // down_left
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->down_left), &(rhs->down_left)))
  {
    return false;
  }
  // down_right
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->down_right), &(rhs->down_right)))
  {
    return false;
  }
  return true;
}

bool
cg_interfaces__msg__RobotSensors__copy(
  const cg_interfaces__msg__RobotSensors * input,
  cg_interfaces__msg__RobotSensors * output)
{
  if (!input || !output) {
    return false;
  }
  // up
  if (!rosidl_runtime_c__String__copy(
      &(input->up), &(output->up)))
  {
    return false;
  }
  // down
  if (!rosidl_runtime_c__String__copy(
      &(input->down), &(output->down)))
  {
    return false;
  }
  // left
  if (!rosidl_runtime_c__String__copy(
      &(input->left), &(output->left)))
  {
    return false;
  }
  // right
  if (!rosidl_runtime_c__String__copy(
      &(input->right), &(output->right)))
  {
    return false;
  }
  // up_left
  if (!rosidl_runtime_c__String__copy(
      &(input->up_left), &(output->up_left)))
  {
    return false;
  }
  // up_right
  if (!rosidl_runtime_c__String__copy(
      &(input->up_right), &(output->up_right)))
  {
    return false;
  }
  // down_left
  if (!rosidl_runtime_c__String__copy(
      &(input->down_left), &(output->down_left)))
  {
    return false;
  }
  // down_right
  if (!rosidl_runtime_c__String__copy(
      &(input->down_right), &(output->down_right)))
  {
    return false;
  }
  return true;
}

cg_interfaces__msg__RobotSensors *
cg_interfaces__msg__RobotSensors__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__msg__RobotSensors * msg = (cg_interfaces__msg__RobotSensors *)allocator.allocate(sizeof(cg_interfaces__msg__RobotSensors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cg_interfaces__msg__RobotSensors));
  bool success = cg_interfaces__msg__RobotSensors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cg_interfaces__msg__RobotSensors__destroy(cg_interfaces__msg__RobotSensors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cg_interfaces__msg__RobotSensors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cg_interfaces__msg__RobotSensors__Sequence__init(cg_interfaces__msg__RobotSensors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__msg__RobotSensors * data = NULL;

  if (size) {
    data = (cg_interfaces__msg__RobotSensors *)allocator.zero_allocate(size, sizeof(cg_interfaces__msg__RobotSensors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cg_interfaces__msg__RobotSensors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cg_interfaces__msg__RobotSensors__fini(&data[i - 1]);
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
cg_interfaces__msg__RobotSensors__Sequence__fini(cg_interfaces__msg__RobotSensors__Sequence * array)
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
      cg_interfaces__msg__RobotSensors__fini(&array->data[i]);
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

cg_interfaces__msg__RobotSensors__Sequence *
cg_interfaces__msg__RobotSensors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__msg__RobotSensors__Sequence * array = (cg_interfaces__msg__RobotSensors__Sequence *)allocator.allocate(sizeof(cg_interfaces__msg__RobotSensors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cg_interfaces__msg__RobotSensors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cg_interfaces__msg__RobotSensors__Sequence__destroy(cg_interfaces__msg__RobotSensors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cg_interfaces__msg__RobotSensors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cg_interfaces__msg__RobotSensors__Sequence__are_equal(const cg_interfaces__msg__RobotSensors__Sequence * lhs, const cg_interfaces__msg__RobotSensors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cg_interfaces__msg__RobotSensors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__msg__RobotSensors__Sequence__copy(
  const cg_interfaces__msg__RobotSensors__Sequence * input,
  cg_interfaces__msg__RobotSensors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cg_interfaces__msg__RobotSensors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cg_interfaces__msg__RobotSensors * data =
      (cg_interfaces__msg__RobotSensors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cg_interfaces__msg__RobotSensors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cg_interfaces__msg__RobotSensors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cg_interfaces__msg__RobotSensors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
