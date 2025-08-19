// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motion_params_service:srv/MotionParamsService.idl
// generated code does not contain a copyright notice

#ifndef MOTION_PARAMS_SERVICE__SRV__DETAIL__MOTION_PARAMS_SERVICE__STRUCT_H_
#define MOTION_PARAMS_SERVICE__SRV__DETAIL__MOTION_PARAMS_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MotionParamsService in the package motion_params_service.
typedef struct motion_params_service__srv__MotionParamsService_Request
{
  int32_t mode;
  int64_t id;
  uint32_t milliseconds;
  uint32_t attitude_loop_milliseconds_cnt;
  uint32_t speed_loop_milliseconds_cnt;
  bool enable_speed_plan;
  float speed_percent;
  float max_v;
  float max_acc;
  float jerk;
  float wheel_diameter;
  float track_width;
  int32_t pluses_per_revolution;
  int32_t revolutions_per_minute;
  float position_p;
  float position_i;
  float position_d;
  float position_max_integral;
  float attitude_p;
  float attitude_i;
  float attitude_d;
  float attitude_max_integral;
  float line_speed_p;
  float line_speed_i;
  float line_speed_d;
  float line_speed_max_integral;
  float angle_speed_p;
  float angle_speed_i;
  float angle_speed_d;
  float angle_speed_max_integral;
  float left_motor_p;
  float left_motor_i;
  float left_motor_d;
  float left_motor_max_integral;
  float right_motor_p;
  float right_motor_i;
  float right_motor_d;
  float right_motor_max_integral;
  int32_t pin_sda;
  int32_t pin_scl;
  int32_t left_motor_pina;
  int32_t left_motor_pinb;
  int32_t left_motor_pwm;
  int32_t left_encoder_pina;
  int32_t left_encoder_pinb;
  int32_t right_motor_pina;
  int32_t right_motor_pinb;
  int32_t right_motor_pwm;
  int32_t right_encoder_pina;
  int32_t right_encoder_pinb;
} motion_params_service__srv__MotionParamsService_Request;

// Struct for a sequence of motion_params_service__srv__MotionParamsService_Request.
typedef struct motion_params_service__srv__MotionParamsService_Request__Sequence
{
  motion_params_service__srv__MotionParamsService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_params_service__srv__MotionParamsService_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MotionParamsService in the package motion_params_service.
typedef struct motion_params_service__srv__MotionParamsService_Response
{
  int32_t state;
  int64_t id;
  int32_t motion_mode;
  bool enable_speed_plan;
  uint32_t milliseconds;
  uint32_t attitude_loop_milliseconds_cnt;
  uint32_t speed_loop_milliseconds_cnt;
  float speed_percent;
  float max_v;
  float max_acc;
  float jerk;
  float wheel_diameter;
  float track_width;
  int32_t pluses_per_revolution;
  int32_t revolutions_per_minute;
  float position_p;
  float position_i;
  float position_d;
  float position_max_integral;
  float attitude_p;
  float attitude_i;
  float attitude_d;
  float attitude_max_integral;
  float line_speed_p;
  float line_speed_i;
  float line_speed_d;
  float line_speed_max_integral;
  float angle_speed_p;
  float angle_speed_i;
  float angle_speed_d;
  float angle_speed_max_integral;
  float left_motor_p;
  float left_motor_i;
  float left_motor_d;
  float left_motor_max_integral;
  float right_motor_p;
  float right_motor_i;
  float right_motor_d;
  float right_motor_max_integral;
  int32_t pin_sda;
  int32_t pin_scl;
  int32_t left_motor_pina;
  int32_t left_motor_pinb;
  int32_t left_motor_pwm;
  int32_t left_encoder_pina;
  int32_t left_encoder_pinb;
  int32_t right_motor_pina;
  int32_t right_motor_pinb;
  int32_t right_motor_pwm;
  int32_t right_encoder_pina;
  int32_t right_encoder_pinb;
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
} motion_params_service__srv__MotionParamsService_Response;

// Struct for a sequence of motion_params_service__srv__MotionParamsService_Response.
typedef struct motion_params_service__srv__MotionParamsService_Response__Sequence
{
  motion_params_service__srv__MotionParamsService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_params_service__srv__MotionParamsService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTION_PARAMS_SERVICE__SRV__DETAIL__MOTION_PARAMS_SERVICE__STRUCT_H_
