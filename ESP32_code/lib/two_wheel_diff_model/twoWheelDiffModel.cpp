#include "twoWheelDiffModel.h"

TwoWheelDiffModel::TwoWheelDiffModel() {
    speedPlan_ = std::make_shared<SpeedPlan>();
    left_motor_control_ = std::make_shared<MotorControl>();
    right_motor_control_ = std::make_shared<MotorControl>();
    mpu6050_control_ = std::make_shared<MPU6050Control>();

    line_speed_loop_ = std::make_shared<PIDControl>();
    angle_speed_loop_ = std::make_shared<PIDControl>();
    position_loop_ = std::make_shared<PIDControl>();
    attitude_loop_ = std::make_shared<PIDControl>();

    mutex_ = xSemaphoreCreateMutex();

    odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_link");
    odom_msg_.twist.twist.linear.y = 0;
    odom_msg_.twist.twist.linear.z = 0;
    odom_msg_.twist.twist.angular.x = 0;
    odom_msg_.twist.twist.angular.y = 0;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.pose.orientation.x = 0;
    odom_msg_.pose.pose.orientation.y = 0;
}

void TwoWheelDiffModel::init_and_start() {
    // set_motion_mode(0);
    // set_milliseconds(20);
    // set_model_settings(0.065, 0.172, 514, 130);

    // set_mpu6050_pins(18, 19);
    // set_left_motor_control_pins(12, 13, 25, 26, -1);
    // set_right_motor_control_pins(22, 23, 33, 32, -1); // 22

    // set_motor_pid_params(1, 0, 0, 1000);
    // set_attitude_ahead_pid_params(1, 0, 0, 1000);
    // set_attitude_back_pid_params(1, 0, 0, 1000);
    // set_speed_pid_params(1, 0, 0, 1000);
    // set_position_pid_params(1, 0, 0, 1000);

    // set_speed_plan_parms(10, 5, 1);

    // start();

    _load_settings();
    _load_params();

    set_motion_mode(motion_mode_);
    set_milliseconds(milliseconds_);
    set_model_settings(wheel_diameter_, track_width_, pluses_per_revolution_, revolutions_per_minute_); // 225, 230

    set_mpu6050_pins(pin_SDA_, pin_SCL_);
    set_left_motor_control_pins(left_motor_pinA_, left_motor_pinB_, left_encoder_pinA_, left_encoder_pinB_, left_motor_pinPWM_);
    set_right_motor_control_pins(right_motor_pinA_, right_motor_pinB_, right_encoder_pinA_, right_encoder_pinB_, right_motor_pinPWM_); // 22

    set_position_pid_params(position_p_, position_i_, position_d_, position_max_integral_);
    set_attitude_pid_params(attitude_p_, attitude_i_, attitude_d_, attitude_max_integral_);

    set_line_speed_pid_params(line_speed_p_, line_speed_i_, line_speed_d_, line_speed_max_integral_);
    set_angle_speed_pid_params(angle_speed_p_, angle_speed_i_, angle_speed_d_, angle_speed_max_integral_);

    set_left_motor_pid_params(left_motor_p_, left_motor_i_, left_motor_d_, left_motor_max_integral_);
    set_right_motor_pid_params(right_motor_p_, right_motor_i_, right_motor_d_, right_motor_max_integral_);

    set_speed_plan_parms(max_v_, max_acc_, jerk_);

    start();
}

void TwoWheelDiffModel::restart() {
    stop();
    delay(100);
    start();
}

void TwoWheelDiffModel::start() {
    mpu6050_control_->start_task();
    _start_control_timer();
}

void TwoWheelDiffModel::stop() {
    stop_move();
    mpu6050_control_->stop_task();
    _stop_control_timer();
}

void TwoWheelDiffModel::set_motion_mode(int mode) {
    motion_mode_ = mode;
    running_ = false;
    delay(100);
    running_ = true;
}

int TwoWheelDiffModel::get_motion_mode() {
    return motion_mode_;
}

void TwoWheelDiffModel::set_model_settings(float wheel_diameter, float track_width, int pluses_per_revolution, int revolutions_per_minute) {
    wheel_diameter_ = wheel_diameter;
    track_width_ = track_width;
    pluses_per_revolution_ = pluses_per_revolution;
    revolutions_per_minute_ = revolutions_per_minute;

    target_max_v_ = revolutions_per_minute_ * PI * wheel_diameter_ / 60;
    target_max_w_ = target_max_v_ * 2 / track_width_;

    left_motor_control_->set_model_params(wheel_diameter_, track_width_, pluses_per_revolution_, revolutions_per_minute_);
    left_motor_control_->set_max_v(target_max_v_);
    right_motor_control_->set_model_params(wheel_diameter_, track_width_, pluses_per_revolution_, revolutions_per_minute_);
    right_motor_control_->set_max_v(target_max_v_);
}

void TwoWheelDiffModel::set_mpu6050_pins(int pin_SDA, int pin_SCL) {
    pin_SDA_ = pin_SDA;
    pin_SCL_ = pin_SCL;

    mpu6050_control_->set_pins(pin_SDA_, pin_SCL_);
}

void TwoWheelDiffModel::set_left_motor_control_pins(int motor_pinA, int motor_pinB, int encoder_pinA, int encoder_pinB, int motor_pinPWM) {
    left_motor_pinA_ = motor_pinA;
    left_motor_pinB_ = motor_pinB;
    left_encoder_pinA_ = encoder_pinA;
    left_encoder_pinB_ = encoder_pinB;
    left_motor_pinPWM_ = motor_pinPWM;

    left_motor_control_->set_pins(left_motor_pinA_, left_motor_pinB_, left_encoder_pinA_, left_encoder_pinB_, left_motor_pinPWM_);
}
void TwoWheelDiffModel::set_right_motor_control_pins(int motor_AIN1, int motor_AIN2, int encoder_pinA, int encoder_pinB, int motor_pinPWM) {
    right_motor_pinA_ = motor_AIN1;
    right_motor_pinB_ = motor_AIN2;
    right_encoder_pinA_ = encoder_pinA;
    right_encoder_pinB_ = encoder_pinB;
    right_motor_pinPWM_ = motor_pinPWM;

    right_motor_control_->set_pins(right_motor_pinA_, right_motor_pinB_, right_encoder_pinA_, right_encoder_pinB_, right_motor_pinPWM_);
}

void TwoWheelDiffModel::set_milliseconds(uint milliseconds) {
    uint milliseconds_tmp = milliseconds_;
    milliseconds_ = milliseconds;

    mpu6050_control_->set_milliseconds(milliseconds_);

    if (esp_timer_is_active(control_timer_) && milliseconds_tmp != milliseconds_) {
        _stop_control_timer();
    }
    _start_control_timer();
}

uint TwoWheelDiffModel::get_milliseconds() {
    return milliseconds_;
}

void TwoWheelDiffModel::set_loop_period_cnt(uint attitude_loop_period_cnt, uint speed_loop_period_cnt) {
    attitude_loop_period_cnt_ = attitude_loop_period_cnt;
    speed_loop_period_cnt_ = speed_loop_period_cnt;
}

float TwoWheelDiffModel::get_max_speed() {
    return max_v_;
}

void TwoWheelDiffModel::set_speed_percent(float percent) {
    percent = fabs(percent);
    if (percent >= 1)
        percent = 1;
    speed_percent_ = percent;
    max_v_ = target_max_v_ * speed_percent_;
    max_w_ = target_max_w_ * speed_percent_;
}

float TwoWheelDiffModel::get_speed_percent() {
    return speed_percent_;
}

void TwoWheelDiffModel::set_speed_plan_state(bool enable) {
    enable_speed_plan_ = enable;
}

void TwoWheelDiffModel::set_speed_plan_parms(float max_v, float max_acc, float jerk) {
    if (max_v > target_max_v_)
        max_v = target_max_v_;

    max_v_ = max_v;
    max_acc_ = max_acc;
    jerk_ = jerk;

    speed_percent_ = max_v_ / target_max_v_;
    max_w_ = target_max_w_ * speed_percent_;

    speedPlan_->set_maxAcc_and_jerk(max_acc_, jerk_);
}

void TwoWheelDiffModel::set_position_pid_params(float p, float i, float d, float max_total_integral) {
    position_p_ = p;
    position_i_ = i;
    position_d_ = d;
    position_max_integral_ = max_total_integral;

    position_loop_->set_pid(position_p_, position_i_, position_d_);
    position_loop_->set_max_integral(position_max_integral_);
    position_loop_->reset();
}

void TwoWheelDiffModel::set_line_speed_pid_params(float p, float i, float d, float max_total_integral) {
    line_speed_p_ = p;
    line_speed_i_ = i;
    line_speed_d_ = d;
    line_speed_max_integral_ = max_total_integral;

    line_speed_loop_->set_pid(line_speed_p_, line_speed_i_, line_speed_d_);
    line_speed_loop_->set_max_integral(line_speed_max_integral_);
    line_speed_loop_->reset();
}

void TwoWheelDiffModel::set_angle_speed_pid_params(float p, float i, float d, float max_total_integral) {
    angle_speed_p_ = p;
    angle_speed_i_ = i;
    angle_speed_d_ = d;
    angle_speed_max_integral_ = max_total_integral;

    angle_speed_loop_->set_pid(angle_speed_p_, angle_speed_i_, angle_speed_d_);
    angle_speed_loop_->set_max_integral(angle_speed_max_integral_);
    angle_speed_loop_->reset();
}

void TwoWheelDiffModel::set_attitude_pid_params(float p, float i, float d, float max_total_integral) {
    attitude_p_ = p;
    attitude_i_ = i;
    attitude_d_ = d;
    attitude_max_integral_ = max_total_integral;

    attitude_loop_->set_pid(attitude_p_, attitude_i_, attitude_d_);
    attitude_loop_->set_max_integral(attitude_max_integral_);
    attitude_loop_->reset();
}

void TwoWheelDiffModel::set_left_motor_pid_params(float p, float i, float d, float max_total_integral) {
    left_motor_p_ = p;
    left_motor_i_ = i;
    left_motor_d_ = d;
    left_motor_max_integral_ = max_total_integral;

    left_motor_control_->set_pid_params(left_motor_p_, left_motor_i_, left_motor_d_, left_motor_max_integral_);
}

void TwoWheelDiffModel::set_right_motor_pid_params(float p, float i, float d, float max_total_integral) {
    right_motor_p_ = p;
    right_motor_i_ = i;
    right_motor_d_ = d;
    right_motor_max_integral_ = max_total_integral;

    right_motor_control_->set_pid_params(right_motor_p_, right_motor_i_, right_motor_d_, right_motor_max_integral_);
}

void TwoWheelDiffModel::mpu6050_calibrate() {
    mpu6050_control_->stop_task();
    delay(100);
    mpu6050_control_->start_calibration();
    mpu6050_control_->start_task();
}

void TwoWheelDiffModel::mpu6050_set_offset() {
    mpu6050_control_->set_offset();
}

void TwoWheelDiffModel::_start_control_timer() {
    if (!control_timer_) {
        esp_timer_create_args_t timer_args = {
            .callback = &control_loop,
            .arg = this,
            .name = "control_timer"};
        auto ret = esp_timer_create(&timer_args, &control_timer_);
        if (ret != ESP_OK) {
            serial_print("control timer created failed.");
            Serial.print("Timer creation failed:");
            Serial.println(ret);
            control_timer_ = nullptr;
            return;
        } else {
            serial_print("control timer created success!");
        }
    }
    if (!esp_timer_is_active(control_timer_)) {
        uint64_t period = milliseconds_ * 1000;
        auto ret = esp_timer_start_periodic(control_timer_, period);
        if (ret != ESP_OK) {
            serial_print("control timer start failed: " + ret);
        } else {
            serial_print("control timer start success!");
        }
    }
}

void TwoWheelDiffModel::_stop_control_timer() {
    stop_move();
    if (control_timer_ && esp_timer_is_active(control_timer_)) {
        esp_timer_stop(control_timer_);
    }
}

void TwoWheelDiffModel::move_relative_pos(float add_x, float add_y, float add_angle) {
    target_x_ = current_x_ + add_x;
    target_y_ = current_y_ + add_y;
    target_angle_ = current_angle_ + add_angle;
}

void TwoWheelDiffModel::move_absolute_pos(float x, float y, float angle) {
    target_x_ = x;
    target_y_ = y;
    target_angle_ = angle;
}

void TwoWheelDiffModel::brake() {
    running_ = false;
    left_motor_control_->brake();
    right_motor_control_->brake();
    delay(100);
    running_ = true;
}

void TwoWheelDiffModel::move_front() {
    start_move(max_v_, 0);
}

void TwoWheelDiffModel::move_back() {
    start_move(-1 * max_v_, 0);
}

void TwoWheelDiffModel::move_left() {
    start_move(0, max_w_);
}

void TwoWheelDiffModel::move_right() {
    start_move(0, -1 * max_w_);
}

void TwoWheelDiffModel::stop_move() {
    start_move(0, 0);
}

void TwoWheelDiffModel::start_move(float v, float w) {
    running_ = true;
    std::deque<std::pair<float, float>> speed_deque;
    if (enable_speed_plan_) {
        float MIN_V_CHANGE = 0.001;
        std::deque<float> single_wheel_speed_deque;
        float left_target_v = 0, right_target_v = 0;

        auto target_wheels_v = _inverseKinematics(v, w);
        auto current_wheels_v = _inverseKinematics(target_v_, target_w_);

        _fix_speed(target_wheels_v.first);
        _fix_speed(target_wheels_v.second);

        float changed_left_v = target_wheels_v.first - current_wheels_v.first;
        float changed_right_v = target_wheels_v.second - current_wheels_v.second;

        if (fabs(changed_left_v) < MIN_V_CHANGE && fabs(changed_right_v) < MIN_V_CHANGE) {
            speed_deque.push_back(std::pair<float, float>(v, w));
        } else {
            if (fabs(changed_left_v) > fabs(changed_right_v)) {
                single_wheel_speed_deque = speedPlan_->plan(current_wheels_v.first, target_wheels_v.first, milliseconds_ / 1000.0);
                for (auto value : single_wheel_speed_deque) {
                    if (fabs(changed_right_v) < MIN_V_CHANGE) {
                        right_target_v = target_wheels_v.second;
                    } else {
                        right_target_v = (value - current_wheels_v.first) / changed_left_v * changed_right_v + current_wheels_v.second;
                    }
                    speed_deque.push_back(_forwardKinematics(value, right_target_v));
                }
            } else {
                single_wheel_speed_deque = speedPlan_->plan(current_wheels_v.second, target_wheels_v.second, milliseconds_ / 1000.0);
                for (auto value : single_wheel_speed_deque) {
                    if (fabs(changed_left_v) < MIN_V_CHANGE) {
                        left_target_v = target_wheels_v.first;
                    } else {
                        left_target_v = (value - current_wheels_v.second) / changed_right_v * changed_left_v + current_wheels_v.first;
                    }
                    speed_deque.push_back(_forwardKinematics(left_target_v, value));
                }
            }
        }
    } else {
        auto target_wheels_v = _inverseKinematics(v, w);

        _fix_speed(target_wheels_v.first);
        _fix_speed(target_wheels_v.second);

        speed_deque.push_back(_forwardKinematics(target_wheels_v.first, target_wheels_v.second));
    }

    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
        target_speed_deque_ = std::move(speed_deque);
        xSemaphoreGive(mutex_);
    }
}

void TwoWheelDiffModel::reset() {
    target_v_ = 0;
    target_w_ = 0;
    adjust_target_v_ = 0;
    adjust_target_w_ = 0;

    period_cnt_ = 0;
    attitude_loop_dt_ = 0;
    speed_loop_dt_ = 0;

    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
        target_speed_deque_.clear();
        xSemaphoreGive(mutex_);
    }

    attitude_loop_->reset();
    line_speed_loop_->reset();
    angle_speed_loop_->reset();
    position_loop_->reset();
}

void TwoWheelDiffModel::update() {
    time_record_ = esp_timer_get_time();
    dt_ = (time_record_ - last_time_record_) / 1e6;

    // TODO:位置环

    left_motor_control_->update(dt_);
    right_motor_control_->update(dt_);
    _calculateOdomMsg();

    if (running_) {
        period_cnt_ = (period_cnt_ + 1) % (attitude_loop_period_cnt_ * speed_loop_period_cnt_);
        speed_loop_dt_ += dt_;

        if (period_cnt_ % speed_loop_period_cnt_ == 0) {
            if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
                if (!target_speed_deque_.empty()) {
                    auto target_speed = target_speed_deque_.front();
                    target_v_ = target_speed.first;
                    target_w_ = target_speed.second;
                    target_speed_deque_.pop_front();
                }
                xSemaphoreGive(mutex_);
            }

            if (motion_mode_ == MotionMode::DynamicBalanceMode) {
                target_pitch_ = line_speed_loop_->calculate(current_v_, target_v_, speed_loop_dt_);
                adjust_target_w_ = angle_speed_loop_->calculate(current_w_, target_w_, speed_loop_dt_);
            } else {
                target_pitch_ = 0;
                adjust_target_w_ = target_w_;
            }
            speed_loop_dt_ = 0;
        }

        if (period_cnt_ % attitude_loop_period_cnt_ == 0) {
            if (motion_mode_ == MotionMode::NoBalanceMode) {
                adjust_target_v_ = target_v_;
            } else {
                if (fabs(mpu6050_control_->pitch_) >= 45) {
                    reset();
                } else {
                    adjust_target_v_ = attitude_p_ * (target_pitch_ - (mpu6050_control_->pitch_ - mpu6050_control_->pitch_offset_)) + attitude_d_ * mpu6050_control_->gyroY_;
                }
            }
        }
    } else {
        reset();
    }

    target_wheel_v_ = _inverseKinematics(adjust_target_v_, adjust_target_w_);

    left_motor_control_->set_speed(target_wheel_v_.first, dt_, running_);
    right_motor_control_->set_speed(target_wheel_v_.second, dt_, running_);

    left_motor_control_->move();
    right_motor_control_->move();

    last_time_record_ = time_record_;

    if (esp_timer_get_time() - time_record_ >= milliseconds_ * 1000) {
        serial_print("控制处理超时(ms): " + std::to_string((esp_timer_get_time() - time_record_) / 1000.0));
    }
}

void TwoWheelDiffModel::_calculateOdomMsg() {
    auto right_distance = right_motor_control_->get_distance_change();
    auto left_distance = left_motor_control_->get_distance_change();
    auto distance = (right_distance + left_distance) / 2;
    auto angle = (right_distance - left_distance) / track_width_;
    current_x_ += distance * std::cos(current_angle_ + angle / 2.0);
    current_y_ += distance * std::sin(current_angle_ + angle / 2.0);
    current_angle_ += angle;
    current_angle_ = std::atan2(std::sin(current_angle_), std::cos(current_angle_));

    auto speed = _forwardKinematics(left_distance / dt_, right_distance / dt_);
    current_v_ = speed.first;
    current_w_ = speed.second;
}

void TwoWheelDiffModel::_fix_speed(float &v) {
    if (v > max_v_)
        v = max_v_;
    else if (v < -1 * max_v_)
        v = -1 * max_v_;
}

std::pair<float, float> TwoWheelDiffModel::_forwardKinematics(float left_v, float right_v) {
    return std::pair<float, float>((left_v + right_v) / 2, (right_v - left_v) / track_width_);
}

std::pair<float, float> TwoWheelDiffModel::_inverseKinematics(float v, float w) {
    return std::pair<float, float>(v - w * track_width_ / 2, v + w * track_width_ / 2);
}

void TwoWheelDiffModel::read_params(motion_params_service__srv__MotionParamsService_Response *response) {
    response->milliseconds = milliseconds_;
    response->attitude_loop_milliseconds_cnt = attitude_loop_period_cnt_;
    response->speed_loop_milliseconds_cnt = speed_loop_period_cnt_;

    response->motion_mode = motion_mode_;
    response->speed_percent = speed_percent_;
    response->max_v = max_v_;
    response->max_acc = max_acc_;
    response->jerk = jerk_;

    response->position_p = position_p_;
    response->position_i = position_i_;
    response->position_d = position_d_;
    response->position_max_integral = position_max_integral_;

    response->attitude_p = attitude_p_;
    response->attitude_i = attitude_i_;
    response->attitude_d = attitude_d_;
    response->attitude_max_integral = attitude_max_integral_;

    response->line_speed_p = line_speed_p_;
    response->line_speed_i = line_speed_i_;
    response->line_speed_d = line_speed_d_;
    response->line_speed_max_integral = line_speed_max_integral_;

    response->angle_speed_p = angle_speed_p_;
    response->angle_speed_i = angle_speed_i_;
    response->angle_speed_d = angle_speed_d_;
    response->angle_speed_max_integral = angle_speed_max_integral_;

    response->left_motor_p = left_motor_p_;
    response->left_motor_i = left_motor_i_;
    response->left_motor_d = left_motor_d_;
    response->left_motor_max_integral = left_motor_max_integral_;

    response->right_motor_p = right_motor_p_;
    response->right_motor_i = right_motor_i_;
    response->right_motor_d = right_motor_d_;
    response->right_motor_max_integral = right_motor_max_integral_;

    mpu6050_control_->get_offset(response->accel_offset_x, response->accel_offset_y, response->accel_offset_z, response->gyro_offset_x, response->gyro_offset_y, response->gyro_offset_z);
}

void TwoWheelDiffModel::save_params() {
    preferences_.begin("params", false);
    preferences_.clear();

    preferences_.putUInt("millisec", milliseconds_);
    preferences_.putUInt("attiLoopmillisecCnt", attitude_loop_period_cnt_);
    preferences_.putUInt("spdLoopmillisecCnt", speed_loop_period_cnt_);

    preferences_.putBool("enableSpeedPlan", enable_speed_plan_);

    preferences_.putFloat("maxV", max_v_);
    preferences_.putFloat("maxAcc", max_acc_);
    preferences_.putFloat("jerk", jerk_);

    preferences_.putFloat("positionP", position_p_);
    preferences_.putFloat("positionI", position_i_);
    preferences_.putFloat("positionD", position_d_);
    preferences_.putFloat("positionMaxI", position_max_integral_);

    preferences_.putFloat("attitudeP", attitude_p_);
    preferences_.putFloat("attitudeI", attitude_i_);
    preferences_.putFloat("attitudeD", attitude_d_);
    preferences_.putFloat("attitudeMaxI", attitude_max_integral_);

    preferences_.putFloat("lineSpeedP", line_speed_p_);
    preferences_.putFloat("lineSpeedI", line_speed_i_);
    preferences_.putFloat("lineSpeedD", line_speed_d_);
    preferences_.putFloat("lineSpeedMaxI", line_speed_max_integral_);

    preferences_.putFloat("angleSpeedP", angle_speed_p_);
    preferences_.putFloat("angleSpeedI", angle_speed_i_);
    preferences_.putFloat("angleSpeedD", angle_speed_d_);
    preferences_.putFloat("angleSpeedMaxI", angle_speed_max_integral_);

    preferences_.putFloat("leftMotorP", left_motor_p_);
    preferences_.putFloat("leftMotorI", left_motor_i_);
    preferences_.putFloat("leftMotorD", left_motor_d_);
    preferences_.putFloat("leftMotorMaxI", left_motor_max_integral_);

    preferences_.putFloat("rightMotorP", right_motor_p_);
    preferences_.putFloat("rightMotorI", right_motor_i_);
    preferences_.putFloat("rightMotorD", right_motor_d_);
    preferences_.putFloat("rightMotorMaxI", right_motor_max_integral_);

    preferences_.end();
    serial_print("保存 params 完成!");
}

void TwoWheelDiffModel::_load_params() {
    preferences_.begin("params", true); // 只读模式

    milliseconds_ = preferences_.getUInt("millisec", milliseconds_);
    attitude_loop_period_cnt_ = preferences_.getUInt("attiLoopmillisecCnt", attitude_loop_period_cnt_);
    speed_loop_period_cnt_ = preferences_.getUInt("spdLoopmillisecCnt", speed_loop_period_cnt_);

    enable_speed_plan_ = preferences_.getBool("enableSpeedPlan", enable_speed_plan_);

    max_v_ = preferences_.getFloat("maxV", max_v_);
    max_acc_ = preferences_.getFloat("maxAcc", max_acc_);
    jerk_ = preferences_.getFloat("jerk", jerk_);

    position_p_ = preferences_.getFloat("positionP", position_p_);
    position_i_ = preferences_.getFloat("positionI", position_i_);
    position_d_ = preferences_.getFloat("positionD", position_d_);
    position_max_integral_ = preferences_.getFloat("positionMaxI", position_max_integral_);

    attitude_p_ = preferences_.getFloat("attitudeP", attitude_p_);
    attitude_i_ = preferences_.getFloat("attitudeI", attitude_i_);
    attitude_d_ = preferences_.getFloat("attitudeD", attitude_d_);
    attitude_max_integral_ = preferences_.getFloat("attitudeMaxI", attitude_max_integral_);

    line_speed_p_ = preferences_.getFloat("lineSpeedP", line_speed_p_);
    line_speed_i_ = preferences_.getFloat("lineSpeedI", line_speed_i_);
    line_speed_d_ = preferences_.getFloat("lineSpeedD", line_speed_d_);
    line_speed_max_integral_ = preferences_.getFloat("lineSpeedMaxI", line_speed_max_integral_);

    angle_speed_p_ = preferences_.getFloat("angleSpeedP", angle_speed_p_);
    angle_speed_i_ = preferences_.getFloat("angleSpeedI", angle_speed_i_);
    angle_speed_d_ = preferences_.getFloat("angleSpeedD", angle_speed_d_);
    angle_speed_max_integral_ = preferences_.getFloat("angleSpeedMaxI", angle_speed_max_integral_);

    left_motor_p_ = preferences_.getFloat("leftMotorP", left_motor_p_);
    left_motor_i_ = preferences_.getFloat("leftMotorI", left_motor_i_);
    left_motor_d_ = preferences_.getFloat("leftMotorD", left_motor_d_);
    left_motor_max_integral_ = preferences_.getFloat("leftMotorMaxI", left_motor_max_integral_);

    right_motor_p_ = preferences_.getFloat("rightMotorP", right_motor_p_);
    right_motor_i_ = preferences_.getFloat("rightMotorI", right_motor_i_);
    right_motor_d_ = preferences_.getFloat("rightMotorD", right_motor_d_);
    right_motor_max_integral_ = preferences_.getFloat("rightMotorMaxI", right_motor_max_integral_);

    preferences_.end();
}

void TwoWheelDiffModel::read_settings(motion_params_service__srv__MotionParamsService_Response *response) {
    response->wheel_diameter = wheel_diameter_;
    response->track_width = track_width_;
    response->pluses_per_revolution = pluses_per_revolution_;
    response->revolutions_per_minute = revolutions_per_minute_;

    response->pin_sda = pin_SDA_;
    response->pin_scl = pin_SCL_;

    response->left_motor_pina = left_motor_pinA_;
    response->left_motor_pinb = left_motor_pinB_;
    response->left_motor_pwm = left_motor_pinPWM_;
    response->left_encoder_pina = left_encoder_pinA_;
    response->left_encoder_pinb = left_encoder_pinB_;

    response->right_motor_pina = right_motor_pinA_;
    response->right_motor_pinb = right_motor_pinB_;
    response->right_motor_pwm = right_motor_pinPWM_;
    response->right_encoder_pina = right_encoder_pinA_;
    response->right_encoder_pinb = right_encoder_pinB_;
}

void TwoWheelDiffModel::save_settings() {
    preferences_.begin("settings", false);
    preferences_.clear();

    preferences_.putInt("mode", motion_mode_);

    preferences_.putFloat("wheDia", wheel_diameter_);
    preferences_.putFloat("traWdi", track_width_);
    preferences_.putInt("pluPerRev", pluses_per_revolution_);
    preferences_.putInt("revPerMin", revolutions_per_minute_);

    preferences_.putInt("pinSDA", pin_SDA_);
    preferences_.putInt("pinSCL", pin_SCL_);

    preferences_.putInt("lMoPinA", left_motor_pinA_);
    preferences_.putInt("lMoPinB", left_motor_pinB_);
    preferences_.putInt("lenPinA", left_encoder_pinA_);
    preferences_.putInt("lenPinB", left_encoder_pinB_);
    preferences_.putInt("lMoPWMPin", left_motor_pinPWM_);

    preferences_.putInt("rMoPinA", right_motor_pinA_);
    preferences_.putInt("rMoPinB", right_motor_pinB_);
    preferences_.putInt("renPinA", right_encoder_pinA_);
    preferences_.putInt("renPinB", right_encoder_pinB_);
    preferences_.putInt("rMoPWMPin", right_motor_pinPWM_);
    preferences_.end();
    serial_print("保存 settings 完成!");
}

void TwoWheelDiffModel::_load_settings() {
    preferences_.begin("settings", true); // 只读模式

    motion_mode_ = preferences_.getInt("mode", motion_mode_);

    wheel_diameter_ = preferences_.getFloat("wheDia", wheel_diameter_);
    track_width_ = preferences_.getFloat("traWdi", track_width_);
    pluses_per_revolution_ = preferences_.getInt("pluPerRev", pluses_per_revolution_);
    revolutions_per_minute_ = preferences_.getInt("revPerMin", revolutions_per_minute_);

    pin_SDA_ = preferences_.getInt("pinSDA", pin_SDA_);
    pin_SCL_ = preferences_.getInt("pinSCL", pin_SCL_);

    left_motor_pinA_ = preferences_.getInt("lMoPinA", left_motor_pinA_);
    left_motor_pinB_ = preferences_.getInt("lMoPinB", left_motor_pinB_);
    left_encoder_pinA_ = preferences_.getInt("lenPinA", left_encoder_pinA_);
    left_encoder_pinB_ = preferences_.getInt("lenPinB", left_encoder_pinB_);
    left_motor_pinPWM_ = preferences_.getInt("lMoPWMPin", left_motor_pinPWM_);
    right_motor_pinA_ = preferences_.getInt("rMoPinA", right_motor_pinA_);
    right_motor_pinB_ = preferences_.getInt("rMoPinB", right_motor_pinB_);

    right_encoder_pinA_ = preferences_.getInt("renPinA", right_encoder_pinA_);
    right_encoder_pinB_ = preferences_.getInt("renPinB", right_encoder_pinB_);
    right_motor_pinPWM_ = preferences_.getInt("rMoPWMPin", right_motor_pinPWM_);

    preferences_.end();
}

nav_msgs__msg__Odometry &TwoWheelDiffModel::get_odom_msg() {
    auto stamp = rmw_uros_epoch_millis();
    odom_msg_.header.stamp.sec = stamp / 1000;
    odom_msg_.header.stamp.nanosec = (stamp % 1000 * 1e6);

    odom_msg_.twist.twist.linear.x = current_v_;
    odom_msg_.twist.twist.angular.z = current_w_;

    odom_msg_.pose.pose.position.x = current_x_;
    odom_msg_.pose.pose.position.y = current_y_;

    odom_msg_.pose.pose.orientation.w = std::cos(current_angle_ / 2.0);
    odom_msg_.pose.pose.orientation.z = std::sin(current_angle_ / 2.0);

    return odom_msg_;
}

motion_status_msgs__msg__MotionStatus &TwoWheelDiffModel::get_motion_status_msg() {
    motion_status_msg_.left_current_v = left_motor_control_->get_current_speed();
    motion_status_msg_.left_target_v = left_motor_control_->get_target_speed();

    motion_status_msg_.right_current_v = right_motor_control_->get_current_speed();
    motion_status_msg_.right_target_v = right_motor_control_->get_target_speed();

    motion_status_msg_.merge_current_v = current_v_;
    motion_status_msg_.merge_target_v = target_v_;

    motion_status_msg_.merge_current_w = current_w_;
    motion_status_msg_.merge_target_w = target_w_;

    mpu6050_control_->get_motion_data(motion_status_msg_.yaw, motion_status_msg_.pitch, motion_status_msg_.roll, motion_status_msg_.gyro_x, motion_status_msg_.gyro_y, motion_status_msg_.gyro_z);

    return motion_status_msg_;
}

String TwoWheelDiffModel::get_http_data() {
    float yaw = 0, pitch = 0, roll = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;
    String json = "{";
    json += "\"speed_percent\":" + String(speed_percent_, 3) + ",";
    json += "\"motion_mode\":" + String(motion_mode_) + ",";
    json += "\"current_v\":" + String(current_v_, 3) + ",";
    json += "\"current_w\":" + String(current_w_, 3) + ",";
    json += "\"current_x\":" + String(current_x_, 3) + ",";
    json += "\"current_y\":" + String(current_y_, 3) + ",";
    json += "\"current_angle\":" + String(current_angle_, 3) + ",";

    mpu6050_control_->get_motion_data(yaw, pitch, roll, gyro_x, gyro_y, gyro_z);
    json += "\"yaw\":" + String(yaw, 3) + ",";
    json += "\"pitch\":" + String(pitch, 3) + ",";
    json += "\"roll\":" + String(roll, 3) + ",";
    json += "\"gyro_x\":" + String(gyro_x, 3) + ",";
    json += "\"gyro_y\":" + String(gyro_y, 3) + ",";
    json += "\"gyro_z\":" + String(gyro_z, 3);
    json += "}";

    return json;
}

void control_loop(void *args) {
    auto twoWheelDiffModel = static_cast<TwoWheelDiffModel *>(args);
    twoWheelDiffModel->update();
}