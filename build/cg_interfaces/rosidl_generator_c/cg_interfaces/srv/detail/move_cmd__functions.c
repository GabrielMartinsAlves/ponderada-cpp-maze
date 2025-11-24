// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cg_interfaces:srv/MoveCmd.idl
// generated code does not contain a copyright notice
#include "cg_interfaces/srv/detail/move_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `direction`
#include "rosidl_runtime_c/string_functions.h"

bool
cg_interfaces__srv__MoveCmd_Request__init(cg_interfaces__srv__MoveCmd_Request * msg)
{
  if (!msg) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__init(&msg->direction)) {
    cg_interfaces__srv__MoveCmd_Request__fini(msg);
    return false;
  }
  return true;
}

void
cg_interfaces__srv__MoveCmd_Request__fini(cg_interfaces__srv__MoveCmd_Request * msg)
{
  if (!msg) {
    return;
  }
  // direction
  rosidl_runtime_c__String__fini(&msg->direction);
}

bool
cg_interfaces__srv__MoveCmd_Request__are_equal(const cg_interfaces__srv__MoveCmd_Request * lhs, const cg_interfaces__srv__MoveCmd_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  return true;
}

bool
cg_interfaces__srv__MoveCmd_Request__copy(
  const cg_interfaces__srv__MoveCmd_Request * input,
  cg_interfaces__srv__MoveCmd_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  return true;
}

cg_interfaces__srv__MoveCmd_Request *
cg_interfaces__srv__MoveCmd_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Request * msg = (cg_interfaces__srv__MoveCmd_Request *)allocator.allocate(sizeof(cg_interfaces__srv__MoveCmd_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cg_interfaces__srv__MoveCmd_Request));
  bool success = cg_interfaces__srv__MoveCmd_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cg_interfaces__srv__MoveCmd_Request__destroy(cg_interfaces__srv__MoveCmd_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cg_interfaces__srv__MoveCmd_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cg_interfaces__srv__MoveCmd_Request__Sequence__init(cg_interfaces__srv__MoveCmd_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Request * data = NULL;

  if (size) {
    data = (cg_interfaces__srv__MoveCmd_Request *)allocator.zero_allocate(size, sizeof(cg_interfaces__srv__MoveCmd_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cg_interfaces__srv__MoveCmd_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cg_interfaces__srv__MoveCmd_Request__fini(&data[i - 1]);
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
cg_interfaces__srv__MoveCmd_Request__Sequence__fini(cg_interfaces__srv__MoveCmd_Request__Sequence * array)
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
      cg_interfaces__srv__MoveCmd_Request__fini(&array->data[i]);
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

cg_interfaces__srv__MoveCmd_Request__Sequence *
cg_interfaces__srv__MoveCmd_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Request__Sequence * array = (cg_interfaces__srv__MoveCmd_Request__Sequence *)allocator.allocate(sizeof(cg_interfaces__srv__MoveCmd_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cg_interfaces__srv__MoveCmd_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cg_interfaces__srv__MoveCmd_Request__Sequence__destroy(cg_interfaces__srv__MoveCmd_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cg_interfaces__srv__MoveCmd_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cg_interfaces__srv__MoveCmd_Request__Sequence__are_equal(const cg_interfaces__srv__MoveCmd_Request__Sequence * lhs, const cg_interfaces__srv__MoveCmd_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cg_interfaces__srv__MoveCmd_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__srv__MoveCmd_Request__Sequence__copy(
  const cg_interfaces__srv__MoveCmd_Request__Sequence * input,
  cg_interfaces__srv__MoveCmd_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cg_interfaces__srv__MoveCmd_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cg_interfaces__srv__MoveCmd_Request * data =
      (cg_interfaces__srv__MoveCmd_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cg_interfaces__srv__MoveCmd_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cg_interfaces__srv__MoveCmd_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cg_interfaces__srv__MoveCmd_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
cg_interfaces__srv__MoveCmd_Response__init(cg_interfaces__srv__MoveCmd_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // robot_pos
  // target_pos
  return true;
}

void
cg_interfaces__srv__MoveCmd_Response__fini(cg_interfaces__srv__MoveCmd_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // robot_pos
  // target_pos
}

bool
cg_interfaces__srv__MoveCmd_Response__are_equal(const cg_interfaces__srv__MoveCmd_Response * lhs, const cg_interfaces__srv__MoveCmd_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // robot_pos
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->robot_pos[i] != rhs->robot_pos[i]) {
      return false;
    }
  }
  // target_pos
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->target_pos[i] != rhs->target_pos[i]) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__srv__MoveCmd_Response__copy(
  const cg_interfaces__srv__MoveCmd_Response * input,
  cg_interfaces__srv__MoveCmd_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // robot_pos
  for (size_t i = 0; i < 2; ++i) {
    output->robot_pos[i] = input->robot_pos[i];
  }
  // target_pos
  for (size_t i = 0; i < 2; ++i) {
    output->target_pos[i] = input->target_pos[i];
  }
  return true;
}

cg_interfaces__srv__MoveCmd_Response *
cg_interfaces__srv__MoveCmd_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Response * msg = (cg_interfaces__srv__MoveCmd_Response *)allocator.allocate(sizeof(cg_interfaces__srv__MoveCmd_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cg_interfaces__srv__MoveCmd_Response));
  bool success = cg_interfaces__srv__MoveCmd_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cg_interfaces__srv__MoveCmd_Response__destroy(cg_interfaces__srv__MoveCmd_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cg_interfaces__srv__MoveCmd_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cg_interfaces__srv__MoveCmd_Response__Sequence__init(cg_interfaces__srv__MoveCmd_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Response * data = NULL;

  if (size) {
    data = (cg_interfaces__srv__MoveCmd_Response *)allocator.zero_allocate(size, sizeof(cg_interfaces__srv__MoveCmd_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cg_interfaces__srv__MoveCmd_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cg_interfaces__srv__MoveCmd_Response__fini(&data[i - 1]);
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
cg_interfaces__srv__MoveCmd_Response__Sequence__fini(cg_interfaces__srv__MoveCmd_Response__Sequence * array)
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
      cg_interfaces__srv__MoveCmd_Response__fini(&array->data[i]);
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

cg_interfaces__srv__MoveCmd_Response__Sequence *
cg_interfaces__srv__MoveCmd_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Response__Sequence * array = (cg_interfaces__srv__MoveCmd_Response__Sequence *)allocator.allocate(sizeof(cg_interfaces__srv__MoveCmd_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cg_interfaces__srv__MoveCmd_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cg_interfaces__srv__MoveCmd_Response__Sequence__destroy(cg_interfaces__srv__MoveCmd_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cg_interfaces__srv__MoveCmd_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cg_interfaces__srv__MoveCmd_Response__Sequence__are_equal(const cg_interfaces__srv__MoveCmd_Response__Sequence * lhs, const cg_interfaces__srv__MoveCmd_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cg_interfaces__srv__MoveCmd_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__srv__MoveCmd_Response__Sequence__copy(
  const cg_interfaces__srv__MoveCmd_Response__Sequence * input,
  cg_interfaces__srv__MoveCmd_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cg_interfaces__srv__MoveCmd_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cg_interfaces__srv__MoveCmd_Response * data =
      (cg_interfaces__srv__MoveCmd_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cg_interfaces__srv__MoveCmd_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cg_interfaces__srv__MoveCmd_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cg_interfaces__srv__MoveCmd_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "cg_interfaces/srv/detail/move_cmd__functions.h"

bool
cg_interfaces__srv__MoveCmd_Event__init(cg_interfaces__srv__MoveCmd_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    cg_interfaces__srv__MoveCmd_Event__fini(msg);
    return false;
  }
  // request
  if (!cg_interfaces__srv__MoveCmd_Request__Sequence__init(&msg->request, 0)) {
    cg_interfaces__srv__MoveCmd_Event__fini(msg);
    return false;
  }
  // response
  if (!cg_interfaces__srv__MoveCmd_Response__Sequence__init(&msg->response, 0)) {
    cg_interfaces__srv__MoveCmd_Event__fini(msg);
    return false;
  }
  return true;
}

void
cg_interfaces__srv__MoveCmd_Event__fini(cg_interfaces__srv__MoveCmd_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  cg_interfaces__srv__MoveCmd_Request__Sequence__fini(&msg->request);
  // response
  cg_interfaces__srv__MoveCmd_Response__Sequence__fini(&msg->response);
}

bool
cg_interfaces__srv__MoveCmd_Event__are_equal(const cg_interfaces__srv__MoveCmd_Event * lhs, const cg_interfaces__srv__MoveCmd_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!cg_interfaces__srv__MoveCmd_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!cg_interfaces__srv__MoveCmd_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
cg_interfaces__srv__MoveCmd_Event__copy(
  const cg_interfaces__srv__MoveCmd_Event * input,
  cg_interfaces__srv__MoveCmd_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!cg_interfaces__srv__MoveCmd_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!cg_interfaces__srv__MoveCmd_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

cg_interfaces__srv__MoveCmd_Event *
cg_interfaces__srv__MoveCmd_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Event * msg = (cg_interfaces__srv__MoveCmd_Event *)allocator.allocate(sizeof(cg_interfaces__srv__MoveCmd_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cg_interfaces__srv__MoveCmd_Event));
  bool success = cg_interfaces__srv__MoveCmd_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cg_interfaces__srv__MoveCmd_Event__destroy(cg_interfaces__srv__MoveCmd_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cg_interfaces__srv__MoveCmd_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cg_interfaces__srv__MoveCmd_Event__Sequence__init(cg_interfaces__srv__MoveCmd_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Event * data = NULL;

  if (size) {
    data = (cg_interfaces__srv__MoveCmd_Event *)allocator.zero_allocate(size, sizeof(cg_interfaces__srv__MoveCmd_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cg_interfaces__srv__MoveCmd_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cg_interfaces__srv__MoveCmd_Event__fini(&data[i - 1]);
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
cg_interfaces__srv__MoveCmd_Event__Sequence__fini(cg_interfaces__srv__MoveCmd_Event__Sequence * array)
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
      cg_interfaces__srv__MoveCmd_Event__fini(&array->data[i]);
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

cg_interfaces__srv__MoveCmd_Event__Sequence *
cg_interfaces__srv__MoveCmd_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cg_interfaces__srv__MoveCmd_Event__Sequence * array = (cg_interfaces__srv__MoveCmd_Event__Sequence *)allocator.allocate(sizeof(cg_interfaces__srv__MoveCmd_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cg_interfaces__srv__MoveCmd_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cg_interfaces__srv__MoveCmd_Event__Sequence__destroy(cg_interfaces__srv__MoveCmd_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cg_interfaces__srv__MoveCmd_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cg_interfaces__srv__MoveCmd_Event__Sequence__are_equal(const cg_interfaces__srv__MoveCmd_Event__Sequence * lhs, const cg_interfaces__srv__MoveCmd_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cg_interfaces__srv__MoveCmd_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cg_interfaces__srv__MoveCmd_Event__Sequence__copy(
  const cg_interfaces__srv__MoveCmd_Event__Sequence * input,
  cg_interfaces__srv__MoveCmd_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cg_interfaces__srv__MoveCmd_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cg_interfaces__srv__MoveCmd_Event * data =
      (cg_interfaces__srv__MoveCmd_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cg_interfaces__srv__MoveCmd_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cg_interfaces__srv__MoveCmd_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cg_interfaces__srv__MoveCmd_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
