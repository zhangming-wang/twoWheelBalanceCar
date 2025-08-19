// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motion_params_service:srv/MotionParamsService.idl
// generated code does not contain a copyright notice
#include "motion_params_service/srv/detail/motion_params_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
motion_params_service__srv__MotionParamsService_Request__init(motion_params_service__srv__MotionParamsService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // id
  // milliseconds
  // attitude_loop_milliseconds_cnt
  // speed_loop_milliseconds_cnt
  // enable_speed_plan
  // speed_percent
  // max_v
  // max_acc
  // jerk
  // wheel_diameter
  // track_width
  // pluses_per_revolution
  // revolutions_per_minute
  // position_p
  // position_i
  // position_d
  // position_max_integral
  // attitude_p
  // attitude_i
  // attitude_d
  // attitude_max_integral
  // line_speed_p
  // line_speed_i
  // line_speed_d
  // line_speed_max_integral
  // angle_speed_p
  // angle_speed_i
  // angle_speed_d
  // angle_speed_max_integral
  // left_motor_p
  // left_motor_i
  // left_motor_d
  // left_motor_max_integral
  // right_motor_p
  // right_motor_i
  // right_motor_d
  // right_motor_max_integral
  // pin_sda
  // pin_scl
  // left_motor_pina
  // left_motor_pinb
  // left_motor_pwm
  // left_encoder_pina
  // left_encoder_pinb
  // right_motor_pina
  // right_motor_pinb
  // right_motor_pwm
  // right_encoder_pina
  // right_encoder_pinb
  return true;
}

void
motion_params_service__srv__MotionParamsService_Request__fini(motion_params_service__srv__MotionParamsService_Request * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // id
  // milliseconds
  // attitude_loop_milliseconds_cnt
  // speed_loop_milliseconds_cnt
  // enable_speed_plan
  // speed_percent
  // max_v
  // max_acc
  // jerk
  // wheel_diameter
  // track_width
  // pluses_per_revolution
  // revolutions_per_minute
  // position_p
  // position_i
  // position_d
  // position_max_integral
  // attitude_p
  // attitude_i
  // attitude_d
  // attitude_max_integral
  // line_speed_p
  // line_speed_i
  // line_speed_d
  // line_speed_max_integral
  // angle_speed_p
  // angle_speed_i
  // angle_speed_d
  // angle_speed_max_integral
  // left_motor_p
  // left_motor_i
  // left_motor_d
  // left_motor_max_integral
  // right_motor_p
  // right_motor_i
  // right_motor_d
  // right_motor_max_integral
  // pin_sda
  // pin_scl
  // left_motor_pina
  // left_motor_pinb
  // left_motor_pwm
  // left_encoder_pina
  // left_encoder_pinb
  // right_motor_pina
  // right_motor_pinb
  // right_motor_pwm
  // right_encoder_pina
  // right_encoder_pinb
}

bool
motion_params_service__srv__MotionParamsService_Request__are_equal(const motion_params_service__srv__MotionParamsService_Request * lhs, const motion_params_service__srv__MotionParamsService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // milliseconds
  if (lhs->milliseconds != rhs->milliseconds) {
    return false;
  }
  // attitude_loop_milliseconds_cnt
  if (lhs->attitude_loop_milliseconds_cnt != rhs->attitude_loop_milliseconds_cnt) {
    return false;
  }
  // speed_loop_milliseconds_cnt
  if (lhs->speed_loop_milliseconds_cnt != rhs->speed_loop_milliseconds_cnt) {
    return false;
  }
  // enable_speed_plan
  if (lhs->enable_speed_plan != rhs->enable_speed_plan) {
    return false;
  }
  // speed_percent
  if (lhs->speed_percent != rhs->speed_percent) {
    return false;
  }
  // max_v
  if (lhs->max_v != rhs->max_v) {
    return false;
  }
  // max_acc
  if (lhs->max_acc != rhs->max_acc) {
    return false;
  }
  // jerk
  if (lhs->jerk != rhs->jerk) {
    return false;
  }
  // wheel_diameter
  if (lhs->wheel_diameter != rhs->wheel_diameter) {
    return false;
  }
  // track_width
  if (lhs->track_width != rhs->track_width) {
    return false;
  }
  // pluses_per_revolution
  if (lhs->pluses_per_revolution != rhs->pluses_per_revolution) {
    return false;
  }
  // revolutions_per_minute
  if (lhs->revolutions_per_minute != rhs->revolutions_per_minute) {
    return false;
  }
  // position_p
  if (lhs->position_p != rhs->position_p) {
    return false;
  }
  // position_i
  if (lhs->position_i != rhs->position_i) {
    return false;
  }
  // position_d
  if (lhs->position_d != rhs->position_d) {
    return false;
  }
  // position_max_integral
  if (lhs->position_max_integral != rhs->position_max_integral) {
    return false;
  }
  // attitude_p
  if (lhs->attitude_p != rhs->attitude_p) {
    return false;
  }
  // attitude_i
  if (lhs->attitude_i != rhs->attitude_i) {
    return false;
  }
  // attitude_d
  if (lhs->attitude_d != rhs->attitude_d) {
    return false;
  }
  // attitude_max_integral
  if (lhs->attitude_max_integral != rhs->attitude_max_integral) {
    return false;
  }
  // line_speed_p
  if (lhs->line_speed_p != rhs->line_speed_p) {
    return false;
  }
  // line_speed_i
  if (lhs->line_speed_i != rhs->line_speed_i) {
    return false;
  }
  // line_speed_d
  if (lhs->line_speed_d != rhs->line_speed_d) {
    return false;
  }
  // line_speed_max_integral
  if (lhs->line_speed_max_integral != rhs->line_speed_max_integral) {
    return false;
  }
  // angle_speed_p
  if (lhs->angle_speed_p != rhs->angle_speed_p) {
    return false;
  }
  // angle_speed_i
  if (lhs->angle_speed_i != rhs->angle_speed_i) {
    return false;
  }
  // angle_speed_d
  if (lhs->angle_speed_d != rhs->angle_speed_d) {
    return false;
  }
  // angle_speed_max_integral
  if (lhs->angle_speed_max_integral != rhs->angle_speed_max_integral) {
    return false;
  }
  // left_motor_p
  if (lhs->left_motor_p != rhs->left_motor_p) {
    return false;
  }
  // left_motor_i
  if (lhs->left_motor_i != rhs->left_motor_i) {
    return false;
  }
  // left_motor_d
  if (lhs->left_motor_d != rhs->left_motor_d) {
    return false;
  }
  // left_motor_max_integral
  if (lhs->left_motor_max_integral != rhs->left_motor_max_integral) {
    return false;
  }
  // right_motor_p
  if (lhs->right_motor_p != rhs->right_motor_p) {
    return false;
  }
  // right_motor_i
  if (lhs->right_motor_i != rhs->right_motor_i) {
    return false;
  }
  // right_motor_d
  if (lhs->right_motor_d != rhs->right_motor_d) {
    return false;
  }
  // right_motor_max_integral
  if (lhs->right_motor_max_integral != rhs->right_motor_max_integral) {
    return false;
  }
  // pin_sda
  if (lhs->pin_sda != rhs->pin_sda) {
    return false;
  }
  // pin_scl
  if (lhs->pin_scl != rhs->pin_scl) {
    return false;
  }
  // left_motor_pina
  if (lhs->left_motor_pina != rhs->left_motor_pina) {
    return false;
  }
  // left_motor_pinb
  if (lhs->left_motor_pinb != rhs->left_motor_pinb) {
    return false;
  }
  // left_motor_pwm
  if (lhs->left_motor_pwm != rhs->left_motor_pwm) {
    return false;
  }
  // left_encoder_pina
  if (lhs->left_encoder_pina != rhs->left_encoder_pina) {
    return false;
  }
  // left_encoder_pinb
  if (lhs->left_encoder_pinb != rhs->left_encoder_pinb) {
    return false;
  }
  // right_motor_pina
  if (lhs->right_motor_pina != rhs->right_motor_pina) {
    return false;
  }
  // right_motor_pinb
  if (lhs->right_motor_pinb != rhs->right_motor_pinb) {
    return false;
  }
  // right_motor_pwm
  if (lhs->right_motor_pwm != rhs->right_motor_pwm) {
    return false;
  }
  // right_encoder_pina
  if (lhs->right_encoder_pina != rhs->right_encoder_pina) {
    return false;
  }
  // right_encoder_pinb
  if (lhs->right_encoder_pinb != rhs->right_encoder_pinb) {
    return false;
  }
  return true;
}

bool
motion_params_service__srv__MotionParamsService_Request__copy(
  const motion_params_service__srv__MotionParamsService_Request * input,
  motion_params_service__srv__MotionParamsService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // id
  output->id = input->id;
  // milliseconds
  output->milliseconds = input->milliseconds;
  // attitude_loop_milliseconds_cnt
  output->attitude_loop_milliseconds_cnt = input->attitude_loop_milliseconds_cnt;
  // speed_loop_milliseconds_cnt
  output->speed_loop_milliseconds_cnt = input->speed_loop_milliseconds_cnt;
  // enable_speed_plan
  output->enable_speed_plan = input->enable_speed_plan;
  // speed_percent
  output->speed_percent = input->speed_percent;
  // max_v
  output->max_v = input->max_v;
  // max_acc
  output->max_acc = input->max_acc;
  // jerk
  output->jerk = input->jerk;
  // wheel_diameter
  output->wheel_diameter = input->wheel_diameter;
  // track_width
  output->track_width = input->track_width;
  // pluses_per_revolution
  output->pluses_per_revolution = input->pluses_per_revolution;
  // revolutions_per_minute
  output->revolutions_per_minute = input->revolutions_per_minute;
  // position_p
  output->position_p = input->position_p;
  // position_i
  output->position_i = input->position_i;
  // position_d
  output->position_d = input->position_d;
  // position_max_integral
  output->position_max_integral = input->position_max_integral;
  // attitude_p
  output->attitude_p = input->attitude_p;
  // attitude_i
  output->attitude_i = input->attitude_i;
  // attitude_d
  output->attitude_d = input->attitude_d;
  // attitude_max_integral
  output->attitude_max_integral = input->attitude_max_integral;
  // line_speed_p
  output->line_speed_p = input->line_speed_p;
  // line_speed_i
  output->line_speed_i = input->line_speed_i;
  // line_speed_d
  output->line_speed_d = input->line_speed_d;
  // line_speed_max_integral
  output->line_speed_max_integral = input->line_speed_max_integral;
  // angle_speed_p
  output->angle_speed_p = input->angle_speed_p;
  // angle_speed_i
  output->angle_speed_i = input->angle_speed_i;
  // angle_speed_d
  output->angle_speed_d = input->angle_speed_d;
  // angle_speed_max_integral
  output->angle_speed_max_integral = input->angle_speed_max_integral;
  // left_motor_p
  output->left_motor_p = input->left_motor_p;
  // left_motor_i
  output->left_motor_i = input->left_motor_i;
  // left_motor_d
  output->left_motor_d = input->left_motor_d;
  // left_motor_max_integral
  output->left_motor_max_integral = input->left_motor_max_integral;
  // right_motor_p
  output->right_motor_p = input->right_motor_p;
  // right_motor_i
  output->right_motor_i = input->right_motor_i;
  // right_motor_d
  output->right_motor_d = input->right_motor_d;
  // right_motor_max_integral
  output->right_motor_max_integral = input->right_motor_max_integral;
  // pin_sda
  output->pin_sda = input->pin_sda;
  // pin_scl
  output->pin_scl = input->pin_scl;
  // left_motor_pina
  output->left_motor_pina = input->left_motor_pina;
  // left_motor_pinb
  output->left_motor_pinb = input->left_motor_pinb;
  // left_motor_pwm
  output->left_motor_pwm = input->left_motor_pwm;
  // left_encoder_pina
  output->left_encoder_pina = input->left_encoder_pina;
  // left_encoder_pinb
  output->left_encoder_pinb = input->left_encoder_pinb;
  // right_motor_pina
  output->right_motor_pina = input->right_motor_pina;
  // right_motor_pinb
  output->right_motor_pinb = input->right_motor_pinb;
  // right_motor_pwm
  output->right_motor_pwm = input->right_motor_pwm;
  // right_encoder_pina
  output->right_encoder_pina = input->right_encoder_pina;
  // right_encoder_pinb
  output->right_encoder_pinb = input->right_encoder_pinb;
  return true;
}

motion_params_service__srv__MotionParamsService_Request *
motion_params_service__srv__MotionParamsService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_params_service__srv__MotionParamsService_Request * msg = (motion_params_service__srv__MotionParamsService_Request *)allocator.allocate(sizeof(motion_params_service__srv__MotionParamsService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_params_service__srv__MotionParamsService_Request));
  bool success = motion_params_service__srv__MotionParamsService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_params_service__srv__MotionParamsService_Request__destroy(motion_params_service__srv__MotionParamsService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_params_service__srv__MotionParamsService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_params_service__srv__MotionParamsService_Request__Sequence__init(motion_params_service__srv__MotionParamsService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_params_service__srv__MotionParamsService_Request * data = NULL;

  if (size) {
    data = (motion_params_service__srv__MotionParamsService_Request *)allocator.zero_allocate(size, sizeof(motion_params_service__srv__MotionParamsService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_params_service__srv__MotionParamsService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_params_service__srv__MotionParamsService_Request__fini(&data[i - 1]);
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
motion_params_service__srv__MotionParamsService_Request__Sequence__fini(motion_params_service__srv__MotionParamsService_Request__Sequence * array)
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
      motion_params_service__srv__MotionParamsService_Request__fini(&array->data[i]);
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

motion_params_service__srv__MotionParamsService_Request__Sequence *
motion_params_service__srv__MotionParamsService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_params_service__srv__MotionParamsService_Request__Sequence * array = (motion_params_service__srv__MotionParamsService_Request__Sequence *)allocator.allocate(sizeof(motion_params_service__srv__MotionParamsService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_params_service__srv__MotionParamsService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_params_service__srv__MotionParamsService_Request__Sequence__destroy(motion_params_service__srv__MotionParamsService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_params_service__srv__MotionParamsService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_params_service__srv__MotionParamsService_Request__Sequence__are_equal(const motion_params_service__srv__MotionParamsService_Request__Sequence * lhs, const motion_params_service__srv__MotionParamsService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_params_service__srv__MotionParamsService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_params_service__srv__MotionParamsService_Request__Sequence__copy(
  const motion_params_service__srv__MotionParamsService_Request__Sequence * input,
  motion_params_service__srv__MotionParamsService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_params_service__srv__MotionParamsService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_params_service__srv__MotionParamsService_Request * data =
      (motion_params_service__srv__MotionParamsService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_params_service__srv__MotionParamsService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_params_service__srv__MotionParamsService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_params_service__srv__MotionParamsService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
motion_params_service__srv__MotionParamsService_Response__init(motion_params_service__srv__MotionParamsService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // state
  // id
  // motion_mode
  // enable_speed_plan
  // milliseconds
  // attitude_loop_milliseconds_cnt
  // speed_loop_milliseconds_cnt
  // speed_percent
  // max_v
  // max_acc
  // jerk
  // wheel_diameter
  // track_width
  // pluses_per_revolution
  // revolutions_per_minute
  // position_p
  // position_i
  // position_d
  // position_max_integral
  // attitude_p
  // attitude_i
  // attitude_d
  // attitude_max_integral
  // line_speed_p
  // line_speed_i
  // line_speed_d
  // line_speed_max_integral
  // angle_speed_p
  // angle_speed_i
  // angle_speed_d
  // angle_speed_max_integral
  // left_motor_p
  // left_motor_i
  // left_motor_d
  // left_motor_max_integral
  // right_motor_p
  // right_motor_i
  // right_motor_d
  // right_motor_max_integral
  // pin_sda
  // pin_scl
  // left_motor_pina
  // left_motor_pinb
  // left_motor_pwm
  // left_encoder_pina
  // left_encoder_pinb
  // right_motor_pina
  // right_motor_pinb
  // right_motor_pwm
  // right_encoder_pina
  // right_encoder_pinb
  // accel_offset_x
  // accel_offset_y
  // accel_offset_z
  // gyro_offset_x
  // gyro_offset_y
  // gyro_offset_z
  return true;
}

void
motion_params_service__srv__MotionParamsService_Response__fini(motion_params_service__srv__MotionParamsService_Response * msg)
{
  if (!msg) {
    return;
  }
  // state
  // id
  // motion_mode
  // enable_speed_plan
  // milliseconds
  // attitude_loop_milliseconds_cnt
  // speed_loop_milliseconds_cnt
  // speed_percent
  // max_v
  // max_acc
  // jerk
  // wheel_diameter
  // track_width
  // pluses_per_revolution
  // revolutions_per_minute
  // position_p
  // position_i
  // position_d
  // position_max_integral
  // attitude_p
  // attitude_i
  // attitude_d
  // attitude_max_integral
  // line_speed_p
  // line_speed_i
  // line_speed_d
  // line_speed_max_integral
  // angle_speed_p
  // angle_speed_i
  // angle_speed_d
  // angle_speed_max_integral
  // left_motor_p
  // left_motor_i
  // left_motor_d
  // left_motor_max_integral
  // right_motor_p
  // right_motor_i
  // right_motor_d
  // right_motor_max_integral
  // pin_sda
  // pin_scl
  // left_motor_pina
  // left_motor_pinb
  // left_motor_pwm
  // left_encoder_pina
  // left_encoder_pinb
  // right_motor_pina
  // right_motor_pinb
  // right_motor_pwm
  // right_encoder_pina
  // right_encoder_pinb
  // accel_offset_x
  // accel_offset_y
  // accel_offset_z
  // gyro_offset_x
  // gyro_offset_y
  // gyro_offset_z
}

bool
motion_params_service__srv__MotionParamsService_Response__are_equal(const motion_params_service__srv__MotionParamsService_Response * lhs, const motion_params_service__srv__MotionParamsService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // motion_mode
  if (lhs->motion_mode != rhs->motion_mode) {
    return false;
  }
  // enable_speed_plan
  if (lhs->enable_speed_plan != rhs->enable_speed_plan) {
    return false;
  }
  // milliseconds
  if (lhs->milliseconds != rhs->milliseconds) {
    return false;
  }
  // attitude_loop_milliseconds_cnt
  if (lhs->attitude_loop_milliseconds_cnt != rhs->attitude_loop_milliseconds_cnt) {
    return false;
  }
  // speed_loop_milliseconds_cnt
  if (lhs->speed_loop_milliseconds_cnt != rhs->speed_loop_milliseconds_cnt) {
    return false;
  }
  // speed_percent
  if (lhs->speed_percent != rhs->speed_percent) {
    return false;
  }
  // max_v
  if (lhs->max_v != rhs->max_v) {
    return false;
  }
  // max_acc
  if (lhs->max_acc != rhs->max_acc) {
    return false;
  }
  // jerk
  if (lhs->jerk != rhs->jerk) {
    return false;
  }
  // wheel_diameter
  if (lhs->wheel_diameter != rhs->wheel_diameter) {
    return false;
  }
  // track_width
  if (lhs->track_width != rhs->track_width) {
    return false;
  }
  // pluses_per_revolution
  if (lhs->pluses_per_revolution != rhs->pluses_per_revolution) {
    return false;
  }
  // revolutions_per_minute
  if (lhs->revolutions_per_minute != rhs->revolutions_per_minute) {
    return false;
  }
  // position_p
  if (lhs->position_p != rhs->position_p) {
    return false;
  }
  // position_i
  if (lhs->position_i != rhs->position_i) {
    return false;
  }
  // position_d
  if (lhs->position_d != rhs->position_d) {
    return false;
  }
  // position_max_integral
  if (lhs->position_max_integral != rhs->position_max_integral) {
    return false;
  }
  // attitude_p
  if (lhs->attitude_p != rhs->attitude_p) {
    return false;
  }
  // attitude_i
  if (lhs->attitude_i != rhs->attitude_i) {
    return false;
  }
  // attitude_d
  if (lhs->attitude_d != rhs->attitude_d) {
    return false;
  }
  // attitude_max_integral
  if (lhs->attitude_max_integral != rhs->attitude_max_integral) {
    return false;
  }
  // line_speed_p
  if (lhs->line_speed_p != rhs->line_speed_p) {
    return false;
  }
  // line_speed_i
  if (lhs->line_speed_i != rhs->line_speed_i) {
    return false;
  }
  // line_speed_d
  if (lhs->line_speed_d != rhs->line_speed_d) {
    return false;
  }
  // line_speed_max_integral
  if (lhs->line_speed_max_integral != rhs->line_speed_max_integral) {
    return false;
  }
  // angle_speed_p
  if (lhs->angle_speed_p != rhs->angle_speed_p) {
    return false;
  }
  // angle_speed_i
  if (lhs->angle_speed_i != rhs->angle_speed_i) {
    return false;
  }
  // angle_speed_d
  if (lhs->angle_speed_d != rhs->angle_speed_d) {
    return false;
  }
  // angle_speed_max_integral
  if (lhs->angle_speed_max_integral != rhs->angle_speed_max_integral) {
    return false;
  }
  // left_motor_p
  if (lhs->left_motor_p != rhs->left_motor_p) {
    return false;
  }
  // left_motor_i
  if (lhs->left_motor_i != rhs->left_motor_i) {
    return false;
  }
  // left_motor_d
  if (lhs->left_motor_d != rhs->left_motor_d) {
    return false;
  }
  // left_motor_max_integral
  if (lhs->left_motor_max_integral != rhs->left_motor_max_integral) {
    return false;
  }
  // right_motor_p
  if (lhs->right_motor_p != rhs->right_motor_p) {
    return false;
  }
  // right_motor_i
  if (lhs->right_motor_i != rhs->right_motor_i) {
    return false;
  }
  // right_motor_d
  if (lhs->right_motor_d != rhs->right_motor_d) {
    return false;
  }
  // right_motor_max_integral
  if (lhs->right_motor_max_integral != rhs->right_motor_max_integral) {
    return false;
  }
  // pin_sda
  if (lhs->pin_sda != rhs->pin_sda) {
    return false;
  }
  // pin_scl
  if (lhs->pin_scl != rhs->pin_scl) {
    return false;
  }
  // left_motor_pina
  if (lhs->left_motor_pina != rhs->left_motor_pina) {
    return false;
  }
  // left_motor_pinb
  if (lhs->left_motor_pinb != rhs->left_motor_pinb) {
    return false;
  }
  // left_motor_pwm
  if (lhs->left_motor_pwm != rhs->left_motor_pwm) {
    return false;
  }
  // left_encoder_pina
  if (lhs->left_encoder_pina != rhs->left_encoder_pina) {
    return false;
  }
  // left_encoder_pinb
  if (lhs->left_encoder_pinb != rhs->left_encoder_pinb) {
    return false;
  }
  // right_motor_pina
  if (lhs->right_motor_pina != rhs->right_motor_pina) {
    return false;
  }
  // right_motor_pinb
  if (lhs->right_motor_pinb != rhs->right_motor_pinb) {
    return false;
  }
  // right_motor_pwm
  if (lhs->right_motor_pwm != rhs->right_motor_pwm) {
    return false;
  }
  // right_encoder_pina
  if (lhs->right_encoder_pina != rhs->right_encoder_pina) {
    return false;
  }
  // right_encoder_pinb
  if (lhs->right_encoder_pinb != rhs->right_encoder_pinb) {
    return false;
  }
  // accel_offset_x
  if (lhs->accel_offset_x != rhs->accel_offset_x) {
    return false;
  }
  // accel_offset_y
  if (lhs->accel_offset_y != rhs->accel_offset_y) {
    return false;
  }
  // accel_offset_z
  if (lhs->accel_offset_z != rhs->accel_offset_z) {
    return false;
  }
  // gyro_offset_x
  if (lhs->gyro_offset_x != rhs->gyro_offset_x) {
    return false;
  }
  // gyro_offset_y
  if (lhs->gyro_offset_y != rhs->gyro_offset_y) {
    return false;
  }
  // gyro_offset_z
  if (lhs->gyro_offset_z != rhs->gyro_offset_z) {
    return false;
  }
  return true;
}

bool
motion_params_service__srv__MotionParamsService_Response__copy(
  const motion_params_service__srv__MotionParamsService_Response * input,
  motion_params_service__srv__MotionParamsService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  // id
  output->id = input->id;
  // motion_mode
  output->motion_mode = input->motion_mode;
  // enable_speed_plan
  output->enable_speed_plan = input->enable_speed_plan;
  // milliseconds
  output->milliseconds = input->milliseconds;
  // attitude_loop_milliseconds_cnt
  output->attitude_loop_milliseconds_cnt = input->attitude_loop_milliseconds_cnt;
  // speed_loop_milliseconds_cnt
  output->speed_loop_milliseconds_cnt = input->speed_loop_milliseconds_cnt;
  // speed_percent
  output->speed_percent = input->speed_percent;
  // max_v
  output->max_v = input->max_v;
  // max_acc
  output->max_acc = input->max_acc;
  // jerk
  output->jerk = input->jerk;
  // wheel_diameter
  output->wheel_diameter = input->wheel_diameter;
  // track_width
  output->track_width = input->track_width;
  // pluses_per_revolution
  output->pluses_per_revolution = input->pluses_per_revolution;
  // revolutions_per_minute
  output->revolutions_per_minute = input->revolutions_per_minute;
  // position_p
  output->position_p = input->position_p;
  // position_i
  output->position_i = input->position_i;
  // position_d
  output->position_d = input->position_d;
  // position_max_integral
  output->position_max_integral = input->position_max_integral;
  // attitude_p
  output->attitude_p = input->attitude_p;
  // attitude_i
  output->attitude_i = input->attitude_i;
  // attitude_d
  output->attitude_d = input->attitude_d;
  // attitude_max_integral
  output->attitude_max_integral = input->attitude_max_integral;
  // line_speed_p
  output->line_speed_p = input->line_speed_p;
  // line_speed_i
  output->line_speed_i = input->line_speed_i;
  // line_speed_d
  output->line_speed_d = input->line_speed_d;
  // line_speed_max_integral
  output->line_speed_max_integral = input->line_speed_max_integral;
  // angle_speed_p
  output->angle_speed_p = input->angle_speed_p;
  // angle_speed_i
  output->angle_speed_i = input->angle_speed_i;
  // angle_speed_d
  output->angle_speed_d = input->angle_speed_d;
  // angle_speed_max_integral
  output->angle_speed_max_integral = input->angle_speed_max_integral;
  // left_motor_p
  output->left_motor_p = input->left_motor_p;
  // left_motor_i
  output->left_motor_i = input->left_motor_i;
  // left_motor_d
  output->left_motor_d = input->left_motor_d;
  // left_motor_max_integral
  output->left_motor_max_integral = input->left_motor_max_integral;
  // right_motor_p
  output->right_motor_p = input->right_motor_p;
  // right_motor_i
  output->right_motor_i = input->right_motor_i;
  // right_motor_d
  output->right_motor_d = input->right_motor_d;
  // right_motor_max_integral
  output->right_motor_max_integral = input->right_motor_max_integral;
  // pin_sda
  output->pin_sda = input->pin_sda;
  // pin_scl
  output->pin_scl = input->pin_scl;
  // left_motor_pina
  output->left_motor_pina = input->left_motor_pina;
  // left_motor_pinb
  output->left_motor_pinb = input->left_motor_pinb;
  // left_motor_pwm
  output->left_motor_pwm = input->left_motor_pwm;
  // left_encoder_pina
  output->left_encoder_pina = input->left_encoder_pina;
  // left_encoder_pinb
  output->left_encoder_pinb = input->left_encoder_pinb;
  // right_motor_pina
  output->right_motor_pina = input->right_motor_pina;
  // right_motor_pinb
  output->right_motor_pinb = input->right_motor_pinb;
  // right_motor_pwm
  output->right_motor_pwm = input->right_motor_pwm;
  // right_encoder_pina
  output->right_encoder_pina = input->right_encoder_pina;
  // right_encoder_pinb
  output->right_encoder_pinb = input->right_encoder_pinb;
  // accel_offset_x
  output->accel_offset_x = input->accel_offset_x;
  // accel_offset_y
  output->accel_offset_y = input->accel_offset_y;
  // accel_offset_z
  output->accel_offset_z = input->accel_offset_z;
  // gyro_offset_x
  output->gyro_offset_x = input->gyro_offset_x;
  // gyro_offset_y
  output->gyro_offset_y = input->gyro_offset_y;
  // gyro_offset_z
  output->gyro_offset_z = input->gyro_offset_z;
  return true;
}

motion_params_service__srv__MotionParamsService_Response *
motion_params_service__srv__MotionParamsService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_params_service__srv__MotionParamsService_Response * msg = (motion_params_service__srv__MotionParamsService_Response *)allocator.allocate(sizeof(motion_params_service__srv__MotionParamsService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_params_service__srv__MotionParamsService_Response));
  bool success = motion_params_service__srv__MotionParamsService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_params_service__srv__MotionParamsService_Response__destroy(motion_params_service__srv__MotionParamsService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_params_service__srv__MotionParamsService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_params_service__srv__MotionParamsService_Response__Sequence__init(motion_params_service__srv__MotionParamsService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_params_service__srv__MotionParamsService_Response * data = NULL;

  if (size) {
    data = (motion_params_service__srv__MotionParamsService_Response *)allocator.zero_allocate(size, sizeof(motion_params_service__srv__MotionParamsService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_params_service__srv__MotionParamsService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_params_service__srv__MotionParamsService_Response__fini(&data[i - 1]);
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
motion_params_service__srv__MotionParamsService_Response__Sequence__fini(motion_params_service__srv__MotionParamsService_Response__Sequence * array)
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
      motion_params_service__srv__MotionParamsService_Response__fini(&array->data[i]);
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

motion_params_service__srv__MotionParamsService_Response__Sequence *
motion_params_service__srv__MotionParamsService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_params_service__srv__MotionParamsService_Response__Sequence * array = (motion_params_service__srv__MotionParamsService_Response__Sequence *)allocator.allocate(sizeof(motion_params_service__srv__MotionParamsService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_params_service__srv__MotionParamsService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_params_service__srv__MotionParamsService_Response__Sequence__destroy(motion_params_service__srv__MotionParamsService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_params_service__srv__MotionParamsService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_params_service__srv__MotionParamsService_Response__Sequence__are_equal(const motion_params_service__srv__MotionParamsService_Response__Sequence * lhs, const motion_params_service__srv__MotionParamsService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_params_service__srv__MotionParamsService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_params_service__srv__MotionParamsService_Response__Sequence__copy(
  const motion_params_service__srv__MotionParamsService_Response__Sequence * input,
  motion_params_service__srv__MotionParamsService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_params_service__srv__MotionParamsService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_params_service__srv__MotionParamsService_Response * data =
      (motion_params_service__srv__MotionParamsService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_params_service__srv__MotionParamsService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_params_service__srv__MotionParamsService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_params_service__srv__MotionParamsService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
