#include "pidControl.h"

PIDControl::PIDControl() {
    p_ = 1;
    i_ = 0;
    d_ = 0;
}

PIDControl::PIDControl(float p, float i, float d, float threshold)
    : p_(p), i_(i), d_(d) {
    reset();
}
float PIDControl::calculate(float current_value, float target_value, float dt, bool debug) {
    float current_error = target_value - current_value;
    float output_value = 0;

    total_integral_ += current_error * dt;

    if (total_integral_ > 0 && total_integral_ > max_integral_)
        total_integral_ = max_integral_;

    if (total_integral_ < 0 && total_integral_ < (-1.0 * max_integral_))
        total_integral_ = -1.0 * max_integral_;

    output_value = p_ * current_error + i_ * total_integral_ + d_ * (current_error - last_error_) / dt;

    last_error_ = current_error;

    return output_value;
}

void PIDControl::reset() {
    last_error_ = 0;
    total_integral_ = 0;
}

void PIDControl::set_pid(float p, float i, float d) {
    p_ = p;
    i_ = i;
    d_ = d;
}

void PIDControl::set_max_integral(float max_integral) {
    max_integral_ = max_integral;
}
float PIDControl::get_max_integral() {
    return max_integral_;
}