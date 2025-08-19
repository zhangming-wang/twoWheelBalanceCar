#include "motorControl.h"

MotorControl::MotorControl() {
    motor_ = std::make_shared<Motor>();
    encoder_ = std::make_shared<Encoder>();
    pidControl_ = std::make_shared<PIDControl>();
    motor_->stop();
    pidControl_->reset();
    encoder_->reset();
}

void MotorControl::update(float dt) {
    encoder_->update(dt);
}

void MotorControl::set_pins(int motor_AIN1, int motor_AIN2, int encoder_pinA, int encoder_pinB, int motor_pwmPin) {
    motor_->set_pins(motor_AIN1, motor_AIN2, motor_pwmPin);
    encoder_->set_pins(encoder_pinA, encoder_pinB);
}

void MotorControl::set_model_params(float wheel_diameter, float track_width, int pluses_per_revolution, int revolutions_per_minute) {
    wheel_diameter_ = wheel_diameter;
    track_width_ = track_width;
    pluses_per_revolution_ = pluses_per_revolution;
    revolutions_per_minute_ = revolutions_per_minute;
}

void MotorControl::set_pid_params(float p, float i, float d, float max_integral) {
    pidControl_->set_pid(p, i, d);
    pidControl_->set_max_integral(max_integral);
    pidControl_->reset();
}

void MotorControl::move() {
    motor_->move();
}

void MotorControl::stop() {
    motor_->stop();
}

void MotorControl::brake() {
    motor_->brake();
}

float MotorControl::get_current_speed() {
    return current_v_;
}

float MotorControl::get_current_acc() {
    return current_acc_;
}

float MotorControl::get_target_speed() {
    return target_v_;
}

float MotorControl::get_target_acc() {
    return target_acc_;
}

float MotorControl::get_distance_change() {
    return encoder_->get_count_change() * PI * wheel_diameter_ / pluses_per_revolution_;
}

long MotorControl::get_encoder_count() {
    return encoder_->get_count();
}

void MotorControl::set_max_v(float max_v) {
    max_v_ = max_v;
}

void MotorControl::set_speed(float target_v, float dt, bool pid_adjust) {
    target_v_ = target_v;
    latest_current_v_ = get_distance_change() / dt;
    current_acc_ = (latest_current_v_ - current_v_) / dt;
    current_v_ = latest_current_v_;
    if (pid_adjust) {
        pid_value_ = pidControl_->calculate(current_v_ * 1000, target_v_ * 1000, dt) / 100.0;
        if (fabs(current_v_) < 0.001 && fabs(target_v_) < 0.001) {
            pid_value_ = 0;
            pidControl_->reset();
        }
    } else {
        pid_value_ = target_v_ / max_v_;
        pidControl_->reset();
    }
    motor_->set_speed(pid_value_);
}