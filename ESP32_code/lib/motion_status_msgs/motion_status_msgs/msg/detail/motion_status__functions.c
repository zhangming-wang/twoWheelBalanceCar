// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motion_status_msgs:msg/MotionStatus.idl
// generated code does not contain a copyright notice
#include "motion_status_msgs/msg/detail/motion_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
motion_status_msgs__msg__MotionStatus__init(motion_status_msgs__msg__MotionStatus * msg)
{
  if (!msg) {
    return false;
  }
  // left_current_v
  // left_target_v
  // right_current_v
  // right_target_v
  // merge_current_v
  // merge_target_v
  // merge_current_w
  // merge_target_w
  // yaw
  // pitch
  // roll
  // gyro_x
  // gyro_y
  // gyro_z
  return true;
}

void
motion_status_msgs__msg__MotionStatus__fini(motion_status_msgs__msg__MotionStatus * msg)
{
  if (!msg) {
    return;
  }
  // left_current_v
  // left_target_v
  // right_current_v
  // right_target_v
  // merge_current_v
  // merge_target_v
  // merge_current_w
  // merge_target_w
  // yaw
  // pitch
  // roll
  // gyro_x
  // gyro_y
  // gyro_z
}

bool
motion_status_msgs__msg__MotionStatus__are_equal(const motion_status_msgs__msg__MotionStatus * lhs, const motion_status_msgs__msg__MotionStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_current_v
  if (lhs->left_current_v != rhs->left_current_v) {
    return false;
  }
  // left_target_v
  if (lhs->left_target_v != rhs->left_target_v) {
    return false;
  }
  // right_current_v
  if (lhs->right_current_v != rhs->right_current_v) {
    return false;
  }
  // right_target_v
  if (lhs->right_target_v != rhs->right_target_v) {
    return false;
  }
  // merge_current_v
  if (lhs->merge_current_v != rhs->merge_current_v) {
    return false;
  }
  // merge_target_v
  if (lhs->merge_target_v != rhs->merge_target_v) {
    return false;
  }
  // merge_current_w
  if (lhs->merge_current_w != rhs->merge_current_w) {
    return false;
  }
  // merge_target_w
  if (lhs->merge_target_w != rhs->merge_target_w) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // gyro_x
  if (lhs->gyro_x != rhs->gyro_x) {
    return false;
  }
  // gyro_y
  if (lhs->gyro_y != rhs->gyro_y) {
    return false;
  }
  // gyro_z
  if (lhs->gyro_z != rhs->gyro_z) {
    return false;
  }
  return true;
}

bool
motion_status_msgs__msg__MotionStatus__copy(
  const motion_status_msgs__msg__MotionStatus * input,
  motion_status_msgs__msg__MotionStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // left_current_v
  output->left_current_v = input->left_current_v;
  // left_target_v
  output->left_target_v = input->left_target_v;
  // right_current_v
  output->right_current_v = input->right_current_v;
  // right_target_v
  output->right_target_v = input->right_target_v;
  // merge_current_v
  output->merge_current_v = input->merge_current_v;
  // merge_target_v
  output->merge_target_v = input->merge_target_v;
  // merge_current_w
  output->merge_current_w = input->merge_current_w;
  // merge_target_w
  output->merge_target_w = input->merge_target_w;
  // yaw
  output->yaw = input->yaw;
  // pitch
  output->pitch = input->pitch;
  // roll
  output->roll = input->roll;
  // gyro_x
  output->gyro_x = input->gyro_x;
  // gyro_y
  output->gyro_y = input->gyro_y;
  // gyro_z
  output->gyro_z = input->gyro_z;
  return true;
}

motion_status_msgs__msg__MotionStatus *
motion_status_msgs__msg__MotionStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_status_msgs__msg__MotionStatus * msg = (motion_status_msgs__msg__MotionStatus *)allocator.allocate(sizeof(motion_status_msgs__msg__MotionStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_status_msgs__msg__MotionStatus));
  bool success = motion_status_msgs__msg__MotionStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_status_msgs__msg__MotionStatus__destroy(motion_status_msgs__msg__MotionStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_status_msgs__msg__MotionStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_status_msgs__msg__MotionStatus__Sequence__init(motion_status_msgs__msg__MotionStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_status_msgs__msg__MotionStatus * data = NULL;

  if (size) {
    data = (motion_status_msgs__msg__MotionStatus *)allocator.zero_allocate(size, sizeof(motion_status_msgs__msg__MotionStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_status_msgs__msg__MotionStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_status_msgs__msg__MotionStatus__fini(&data[i - 1]);
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
motion_status_msgs__msg__MotionStatus__Sequence__fini(motion_status_msgs__msg__MotionStatus__Sequence * array)
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
      motion_status_msgs__msg__MotionStatus__fini(&array->data[i]);
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

motion_status_msgs__msg__MotionStatus__Sequence *
motion_status_msgs__msg__MotionStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_status_msgs__msg__MotionStatus__Sequence * array = (motion_status_msgs__msg__MotionStatus__Sequence *)allocator.allocate(sizeof(motion_status_msgs__msg__MotionStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_status_msgs__msg__MotionStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_status_msgs__msg__MotionStatus__Sequence__destroy(motion_status_msgs__msg__MotionStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_status_msgs__msg__MotionStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_status_msgs__msg__MotionStatus__Sequence__are_equal(const motion_status_msgs__msg__MotionStatus__Sequence * lhs, const motion_status_msgs__msg__MotionStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_status_msgs__msg__MotionStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_status_msgs__msg__MotionStatus__Sequence__copy(
  const motion_status_msgs__msg__MotionStatus__Sequence * input,
  motion_status_msgs__msg__MotionStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_status_msgs__msg__MotionStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_status_msgs__msg__MotionStatus * data =
      (motion_status_msgs__msg__MotionStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_status_msgs__msg__MotionStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_status_msgs__msg__MotionStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_status_msgs__msg__MotionStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
