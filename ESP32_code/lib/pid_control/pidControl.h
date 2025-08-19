#pragma once

#include <Arduino.h>
#include <iostream>
#include <math.h>

class PIDControl {
public:
    PIDControl();
    PIDControl(float p, float i, float d, float threshold = 0);

    float calculate(float current_value, float target_value, float dt, bool debug = false);
    void set_pid(float p, float i, float d);
    void set_max_integral(float max_integral);
    float get_max_integral();
    void reset();

private:
    volatile float p_ = 1;
    volatile float i_ = 0;
    volatile float d_ = 0;

    volatile float last_error_ = 0;
    volatile float total_integral_ = 0;
    volatile float max_integral_ = 5000;
};