// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motion_status_msgs:msg/MotionStatus.idl
// generated code does not contain a copyright notice

#ifndef MOTION_STATUS_MSGS__MSG__DETAIL__MOTION_STATUS__STRUCT_H_
#define MOTION_STATUS_MSGS__MSG__DETAIL__MOTION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotionStatus in the package motion_status_msgs.
typedef struct motion_status_msgs__msg__MotionStatus
{
  float left_current_v;
  float left_target_v;
  float right_current_v;
  float right_target_v;
  float merge_current_v;
  float merge_target_v;
  float merge_current_w;
  float merge_target_w;
  float yaw;
  float pitch;
  float roll;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} motion_status_msgs__msg__MotionStatus;

// Struct for a sequence of motion_status_msgs__msg__MotionStatus.
typedef struct motion_status_msgs__msg__MotionStatus__Sequence
{
  motion_status_msgs__msg__MotionStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_status_msgs__msg__MotionStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTION_STATUS_MSGS__MSG__DETAIL__MOTION_STATUS__STRUCT_H_
