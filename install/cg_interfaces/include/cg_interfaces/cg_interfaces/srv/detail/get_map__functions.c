// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cg_interfaces:srv/GetMap.idl
// generated code does not contain a copyright notice
#include "cg_interfaces/srv/detail/get_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
cg_interfaces__srv__GetMap_Request__init(cg_interfaces__srv__GetMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
cg_interfaces__srv__GetMap_Request__fini(cg_interfaces__srv__GetMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
cg_interfaces__srv__GetMap_Request__are_equal(const cg_interfaces__srv__GetMap_Request * lhs, const cg_interfaces__srv__GetMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
cg_interfaces__srv__GetMap_Request__copy(
  const cg_interfaces__srv__GetMap_Request * input,
  cg_interfaces__srv__GetMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

cg_interfaces__srv__GetMap_Request *
cg_interfaces__srv__GetMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__GetMap_Request * msg = (cg_interfaces__srv__GetMap_Request *)allocator.allocate(sizeof(cg_interfaces__srv__GetMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cg_interfaces__srv__GetMap_Request));
  bool success = cg_interfaces__srv__GetMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cg_interfaces__srv__GetMap_Request__destroy(cg_interfaces__srv__GetMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cg_interfaces__srv__GetMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cg_interfaces__srv__GetMap_Request__Sequence__init(cg_interfaces__srv__GetMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__GetMap_Request * data = NULL;

  if (size) {
    data = (cg_interfaces__srv__GetMap_Request *)allocator.zero_allocate(size, sizeof(cg_interfaces__srv__GetMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cg_interfaces__srv__GetMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cg_interfaces__srv__GetMap_Request__fini(&data[i - 1]);
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
cg_interfaces__srv__GetMap_Request__Sequence__fini(cg_interfaces__srv__GetMap_Request__Sequence * array)
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
      cg_interfaces__srv__GetMap_Request__fini(&array->data[i]);
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

cg_interfaces__srv__GetMap_Request__Sequence *
cg_interfaces__srv__GetMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__GetMap_Request__Sequence * array = (cg_interfaces__srv__GetMap_Request__Sequence *)allocator.allocate(sizeof(cg_interfaces__srv__GetMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cg_interfaces__srv__GetMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cg_interfaces__srv__GetMap_Request__Sequence__destroy(cg_interfaces__srv__GetMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cg_interfaces__srv__GetMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cg_interfaces__srv__GetMap_Request__Sequence__are_equal(const cg_interfaces__srv__GetMap_Request__Sequence * lhs, const cg_interfaces__srv__GetMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cg_interfaces__srv__GetMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__srv__GetMap_Request__Sequence__copy(
  const cg_interfaces__srv__GetMap_Request__Sequence * input,
  cg_interfaces__srv__GetMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cg_interfaces__srv__GetMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cg_interfaces__srv__GetMap_Request * data =
      (cg_interfaces__srv__GetMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cg_interfaces__srv__GetMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cg_interfaces__srv__GetMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cg_interfaces__srv__GetMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `occupancy_grid_flattened`
#include "rosidl_runtime_c/string_functions.h"
// Member `occupancy_grid_shape`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
cg_interfaces__srv__GetMap_Response__init(cg_interfaces__srv__GetMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // occupancy_grid_flattened
  if (!rosidl_runtime_c__String__Sequence__init(&msg->occupancy_grid_flattened, 0)) {
    cg_interfaces__srv__GetMap_Response__fini(msg);
    return false;
  }
  // occupancy_grid_shape
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->occupancy_grid_shape, 0)) {
    cg_interfaces__srv__GetMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
cg_interfaces__srv__GetMap_Response__fini(cg_interfaces__srv__GetMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // occupancy_grid_flattened
  rosidl_runtime_c__String__Sequence__fini(&msg->occupancy_grid_flattened);
  // occupancy_grid_shape
  rosidl_runtime_c__uint8__Sequence__fini(&msg->occupancy_grid_shape);
}

bool
cg_interfaces__srv__GetMap_Response__are_equal(const cg_interfaces__srv__GetMap_Response * lhs, const cg_interfaces__srv__GetMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // occupancy_grid_flattened
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->occupancy_grid_flattened), &(rhs->occupancy_grid_flattened)))
  {
    return false;
  }
  // occupancy_grid_shape
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->occupancy_grid_shape), &(rhs->occupancy_grid_shape)))
  {
    return false;
  }
  return true;
}

bool
cg_interfaces__srv__GetMap_Response__copy(
  const cg_interfaces__srv__GetMap_Response * input,
  cg_interfaces__srv__GetMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // occupancy_grid_flattened
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->occupancy_grid_flattened), &(output->occupancy_grid_flattened)))
  {
    return false;
  }
  // occupancy_grid_shape
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->occupancy_grid_shape), &(output->occupancy_grid_shape)))
  {
    return false;
  }
  return true;
}

cg_interfaces__srv__GetMap_Response *
cg_interfaces__srv__GetMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__GetMap_Response * msg = (cg_interfaces__srv__GetMap_Response *)allocator.allocate(sizeof(cg_interfaces__srv__GetMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cg_interfaces__srv__GetMap_Response));
  bool success = cg_interfaces__srv__GetMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cg_interfaces__srv__GetMap_Response__destroy(cg_interfaces__srv__GetMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cg_interfaces__srv__GetMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cg_interfaces__srv__GetMap_Response__Sequence__init(cg_interfaces__srv__GetMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__GetMap_Response * data = NULL;

  if (size) {
    data = (cg_interfaces__srv__GetMap_Response *)allocator.zero_allocate(size, sizeof(cg_interfaces__srv__GetMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cg_interfaces__srv__GetMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cg_interfaces__srv__GetMap_Response__fini(&data[i - 1]);
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
cg_interfaces__srv__GetMap_Response__Sequence__fini(cg_interfaces__srv__GetMap_Response__Sequence * array)
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
      cg_interfaces__srv__GetMap_Response__fini(&array->data[i]);
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

cg_interfaces__srv__GetMap_Response__Sequence *
cg_interfaces__srv__GetMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__GetMap_Response__Sequence * array = (cg_interfaces__srv__GetMap_Response__Sequence *)allocator.allocate(sizeof(cg_interfaces__srv__GetMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cg_interfaces__srv__GetMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cg_interfaces__srv__GetMap_Response__Sequence__destroy(cg_interfaces__srv__GetMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cg_interfaces__srv__GetMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cg_interfaces__srv__GetMap_Response__Sequence__are_equal(const cg_interfaces__srv__GetMap_Response__Sequence * lhs, const cg_interfaces__srv__GetMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cg_interfaces__srv__GetMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__srv__GetMap_Response__Sequence__copy(
  const cg_interfaces__srv__GetMap_Response__Sequence * input,
  cg_interfaces__srv__GetMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cg_interfaces__srv__GetMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cg_interfaces__srv__GetMap_Response * data =
      (cg_interfaces__srv__GetMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cg_interfaces__srv__GetMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cg_interfaces__srv__GetMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cg_interfaces__srv__GetMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
