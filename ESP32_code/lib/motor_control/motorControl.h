#pragma once

#include "encoder.h"
#include "motor.h"
#include "pidControl.h"
#include <deque>
#include <iostream>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MotorControl {
public:
    MotorControl();
    bool init_success();

    void set_pins(int motor_AIN1, int motor_AIN2, int encoder_pinA, int encoder_pinB, int motor_pwmPin = -1);

    void update(float dt);

    void reset();

    void move();
    void stop();
    void brake();

    void set_model_params(float wheel_diameter, float track_width, int pluses_per_revolution, int revolutions_per_minute);
    void set_pid_params(float p, float i, float d, float max_integral);

    void set_max_v(float max_v);

    float get_current_speed();
    float get_current_acc();

    float get_target_speed();
    float get_target_acc();

    float get_distance_change();
    long get_encoder_count();

    void set_speed(float speed, float dt, bool pid_adjust = false);

private:
    float wheel_diameter_ = 0;
    float track_width_ = 0;
    int pluses_per_revolution_ = 0;
    int revolutions_per_minute_ = 0;

    volatile float current_v_ = 0, target_v_ = 0, current_acc_ = 0, target_acc_ = 0;
    volatile float latest_target_v_ = 0, latest_current_v_ = 0, max_v_ = 0;

    float pid_value_ = 0;

    std::shared_ptr<Encoder> encoder_;
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<PIDControl> pidControl_;
};