#pragma once

#include "esp_timer.h"
#include "micro_ros_utilities/string_utilities.h"
#include "motorControl.h"
#include "mpu6050Control.h"
#include "nav_msgs/msg/odometry.h"
#include "rosidl_runtime_c/string_functions.h"
#include "speedPlan.h"
extern "C" {
#include "motion_params_service/srv/motion_params_service.h"
#include "motion_status_msgs/msg/motion_status.h"
}
#include "enum.h"
#include <Arduino.h>
#include <Preferences.h>
#include <deque>
#include <memory>
#include <micro_ros_platformio.h>
#include <time.h>
#include <utility>
#include <vector>

class TwoWheelDiffModel {
    friend void control_loop(void *args);

public:
    TwoWheelDiffModel();

    void init_and_start();

    void start();
    void stop();

    void restart();

    void start_move(float v, float w);
    void stop_move();
    void brake();

    void move_front();
    void move_back();
    void move_left();
    void move_right();

    void move_absolute_pos(float x, float y, float angle);
    void move_relative_pos(float add_x, float add_y, float add_angle);

    String get_http_data();

    void set_motion_mode(int mode);
    int get_motion_mode();

    void set_model_settings(float wheel_diameter, float track_width, int pluses_per_revolution, int revolutions_per_minute);
    void set_mpu6050_pins(int pin_SDA, int pin_SCL);
    void set_left_motor_control_pins(int motor_AIN1, int motor_AIN2, int encoder_pinA, int encoder_pinB, int motor_pwmPin = -1);
    void set_right_motor_control_pins(int motor_AIN1, int motor_AIN2, int encoder_pinA, int encoder_pinB, int motor_pwmPin = -1);

    void set_milliseconds(uint milliseconds);
    uint get_milliseconds();

    void set_loop_period_cnt(uint attitude_loop_period_cnt, uint speed_loop_period_cnt_);

    void set_speed_percent(float percent);
    float get_speed_percent();
    float get_max_speed();

    void set_speed_plan_state(bool enable);

    void set_position_pid_params(float p, float i, float d, float max_total_integral);
    void set_attitude_pid_params(float p, float i, float d, float max_total_integral);

    void set_left_motor_pid_params(float p, float i, float d, float max_total_integral);
    void set_right_motor_pid_params(float p, float i, float d, float max_total_integral);

    void set_line_speed_pid_params(float p, float i, float d, float max_total_integral);
    void set_angle_speed_pid_params(float p, float i, float d, float max_total_integral);

    void set_speed_plan_parms(float max_v, float max_acc, float jerk);

    void mpu6050_calibrate();
    void mpu6050_set_offset();

    void read_settings(motion_params_service__srv__MotionParamsService_Response *response);
    void save_settings();

    void read_params(motion_params_service__srv__MotionParamsService_Response *response);
    void save_params();

    motion_status_msgs__msg__MotionStatus &get_motion_status_msg();
    nav_msgs__msg__Odometry &get_odom_msg();

private:
    //---------需要保存配置---------
    int motion_mode_ = 0;

    float wheel_diameter_ = 10, track_width_ = 10;
    int pluses_per_revolution_ = 10, revolutions_per_minute_ = 10;

    int pin_SDA_ = -1, pin_SCL_ = -1;
    int left_motor_pinA_ = -1, left_motor_pinB_ = -1, left_encoder_pinA_ = -1, left_encoder_pinB_ = -1, left_motor_pinPWM_ = -1;
    int right_motor_pinA_ = -1, right_motor_pinB_ = -1, right_encoder_pinA_ = -1, right_encoder_pinB_ = -1, right_motor_pinPWM_ = -1;

    //---------需要保存参数---------
    volatile uint milliseconds_ = 10, attitude_loop_period_cnt_ = 2, speed_loop_period_cnt_ = 5;

    float max_v_ = 1, max_acc_ = 10, jerk_ = 1, max_w_ = 0;
    float target_max_v_ = 0, target_max_w_ = 0, speed_percent_ = 0, target_pitch_ = 0;

    float position_p_ = 1, position_i_ = 0, position_d_ = 0, position_max_integral_ = 1000;
    float attitude_p_ = 1, attitude_i_ = 0, attitude_d_ = 0, attitude_max_integral_ = 1000;

    float line_speed_p_ = 1, line_speed_i_ = 0, line_speed_d_ = 0, line_speed_max_integral_ = 1000;
    float angle_speed_p_ = 1, angle_speed_i_ = 0, angle_speed_d_ = 0, angle_speed_max_integral_ = 1000;

    float left_motor_p_ = 1, left_motor_i_ = 0, left_motor_d_ = 0, left_motor_max_integral_ = 1000;
    float right_motor_p_ = 1, right_motor_i_ = 0, right_motor_d_ = 0, right_motor_max_integral_ = 1000;

    //---------局部内部参数---------
    uint period_cnt_ = 0;
    volatile bool running_ = false, enable_speed_plan_ = false;

    float dt_ = 0, attitude_loop_dt_ = 0, speed_loop_dt_ = 0;
    int64_t time_record_ = 0, last_time_record_ = 0;

    volatile float current_x_ = 0, current_y_ = 0, current_angle_ = 0, target_x_ = 0, target_y_ = 0, target_angle_ = 0;
    volatile float current_v_ = 0, current_w_ = 0, target_v_ = 0, target_w_ = 0, adjust_target_v_ = 0, adjust_target_w_ = 0;

    std::pair<float, float> target_wheel_v_;
    std::deque<std::pair<float, float>> target_speed_deque_;

    Preferences preferences_;

    std::shared_ptr<MotorControl> left_motor_control_, right_motor_control_;
    std::shared_ptr<MPU6050Control> mpu6050_control_;
    std::shared_ptr<PIDControl> position_loop_, line_speed_loop_, angle_speed_loop_, attitude_loop_;
    std::shared_ptr<SpeedPlan> speedPlan_;

    esp_timer_handle_t control_timer_ = nullptr;

    SemaphoreHandle_t mutex_; // 互斥量句柄

    motion_status_msgs__msg__MotionStatus motion_status_msg_;
    nav_msgs__msg__Odometry odom_msg_;

    std::pair<float, float> _forwardKinematics(float left_v, float right_v);
    std::pair<float, float> _inverseKinematics(float v, float w);

    void _calculateOdomMsg();

    void _load_params();
    void _load_settings();

    void _start_control_timer();
    void _stop_control_timer();

    void _fix_speed(float &v);

    void update();
    void reset();
};

extern TwoWheelDiffModel twoWheelDiffModel;

void control_loop(void *args);
