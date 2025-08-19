#include "microRos.h"

MicroRos::MicroRos(const std::string &node_name, const std::string &wifi_name, const std::string &wifi_passward, const std::string &ip, const uint16_t port) : node_name_(node_name),
                                                                                                                                                               wifi_name_(wifi_name),
                                                                                                                                                               wifi_passward_(wifi_passward),
                                                                                                                                                               ip_(ip),
                                                                                                                                                               port_(port) {

    allocator_ = rcl_get_default_allocator();

    IPAddress agent_ip;
    agent_ip.fromString(ip.c_str());
    locator_.address = agent_ip;
    locator_.port = port;
}

bool MicroRos::is_connected() {
    return connected;
}

void MicroRos::clean() {
    rcl_ret_t ret;
    if (timer_initialized_) {
        ret = rcl_timer_fini(&timer_);
        if (ret != RCL_RET_OK) {
            serial_print("rcl_timer_fini error: " + std::to_string(ret));
        }
        timer_initialized_ = false;
    }
    if (motion_params_service_initialized_) {
        ret = rcl_service_fini(&motion_params_service_, &node_);
        if (ret != RCL_RET_OK) {
            serial_print("rcl_service_fini motion_params_service error: " + std::to_string(ret));
        }
        motion_params_service_initialized_ = false;
    }
    if (cmd_vel_subscription_initialized_) {
        ret = rcl_subscription_fini(&cmd_vel_subscription_, &node_);
        if (ret != RCL_RET_OK) {
            serial_print("rcl_subscription_fini error: " + std::to_string(ret));
        }
        cmd_vel_subscription_initialized_ = false;
    }
    if (motion_status_publisher_initialized_) {
        ret = rcl_publisher_fini(&motion_status_publisher_, &node_);
        if (ret != RCL_RET_OK) {
            serial_print("motion_status_publisher:rcl_publisher_fini error: " + std::to_string(ret));
        }
        motion_status_publisher_initialized_ = false;
    }
    if (odom_publisher_initialized_) {
        ret = rcl_publisher_fini(&odom_publisher_, &node_);
        if (ret != RCL_RET_OK) {
            serial_print("odom_publisher:rcl_publisher_fini error: " + std::to_string(ret));
        }
        odom_publisher_initialized_ = false;
    }
    if (serial_msg_publisher_initialized_) {
        ret = rcl_publisher_fini(&serial_msg_publisher_, &node_);
        if (ret != RCL_RET_OK) {
            serial_print("serial_msg_publisher_:rcl_publisher_fini error: " + std::to_string(ret));
        }
        serial_msg_publisher_initialized_ = false;
    }
    if (executor_initialized_) {
        ret = rclc_executor_fini(&executor);
        if (ret != RCL_RET_OK) {
            serial_print("executor_initialized_ error: " + std::to_string(ret));
        }
        executor_initialized_ = false;
    }
    if (node_initialized_) {
        ret = rcl_node_fini(&node_);
        if (ret != RCL_RET_OK) {
            serial_print("rcl_node_fini error: " + std::to_string(ret));
        }
        node_initialized_ = false;
    }
    if (support_initialized_) {
        ret = rclc_support_fini(&support_);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_support_fini error: " + std::to_string(ret));
        }
        support_initialized_ = false;
    }
}

bool MicroRos::init() {
    clean();
    int num_handles = 4;
    bool init_success = true;
    IPAddress agent_ip;
    rcl_ret_t ret;
    agent_ip.fromString(ip_.c_str());

    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    auto status = rmw_uros_set_custom_transport(
        false,
        (void *)&locator_,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read);

    if (status != 0) {
        serial_print("rmw_uros_set_custom_transport:" + std::to_string(status));
        return false;
    }

    // set_microros_wifi_transports(const_cast<char *>(wifi_name_.c_str()), const_cast<char *>(wifi_passward_.c_str()), agent_ip, port_);

    if (!support_initialized_) {
        ret = rclc_support_init(&support_, 0, NULL, &allocator_);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_support_init:" + std::to_string(ret));
            return false;
        }
        support_initialized_ = true;
    }

    while (!rmw_uros_epoch_synchronized()) {
        rmw_uros_sync_session(1000);
        delay(10);
    }

    if (!node_initialized_) {
        node_ = rcl_get_zero_initialized_node();
        ret = rclc_node_init_default(&node_, node_name_.c_str(), "", &support_);
        if (ret != RCL_RET_OK) {
            serial_print("rcl_get_zero_initialized_node:" + std::to_string(ret));
            return false;
        }
        node_initialized_ = true;
    }

    if (!executor_initialized_) {
        ret = rclc_executor_init(&executor, &support_.context, num_handles, &allocator_);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_executor_init:" + std::to_string(ret));
            return false;
        }
        executor_initialized_ = true;
    }

    if (!cmd_vel_subscription_initialized_) {
        ret = rclc_subscription_init_default(&cmd_vel_subscription_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
        if (ret != RCL_RET_OK) {
            serial_print("rclc_subscription_init_default:" + std::to_string(ret));
            return false;
        } else {
            ret = rclc_executor_add_subscription(&executor, &cmd_vel_subscription_, &msg_cmd_vel, msg_twist_callback, ON_NEW_DATA);
            if (ret != RCL_RET_OK) {
                serial_print("rclc_executor_add_subscription:" + std::to_string(ret));
                ret = rcl_subscription_fini(&cmd_vel_subscription_, &node_);
                if (ret != RCL_RET_OK) {
                    serial_print("rcl_subscription_fini error: " + std::to_string(ret));
                }
                return false;
            }
        }
        cmd_vel_subscription_initialized_ = true;
    }

    if (!odom_publisher_initialized_) {
        ret = rclc_publisher_init_default(&odom_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"); //
        if (ret != RCL_RET_OK) {
            serial_print("odom_publisher_:rclc_publisher_init_default:" + std::to_string(ret));
            return false;
        }
        odom_publisher_initialized_ = true;
    }

    if (!motion_status_publisher_initialized_) {
        ret = rclc_publisher_init_default(&motion_status_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(motion_status_msgs, msg, MotionStatus), "/motion_status_topic"); //
        if (ret != RCL_RET_OK) {
            serial_print("motion_status_publisher_:rclc_publisher_init_default:" + std::to_string(ret));
            return false;
        }
        motion_status_publisher_initialized_ = true;
    }

    if (!serial_msg_publisher_initialized_) {
        ret = rclc_publisher_init_default(&serial_msg_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/serial_msg_topic"); //
        if (ret != RCL_RET_OK) {
            serial_print("serial_msg_publisher_:rclc_publisher_init_default:" + std::to_string(ret));
            return false;
        }
        serial_msg_publisher_initialized_ = true;
    }

    if (!timer_initialized_) {
        ret = rclc_timer_init_default(&timer_, &support_, RCL_MS_TO_NS(twoWheelDiffModel.get_milliseconds()), timer_callback);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_timer_init_default:" + std::to_string(ret));
            return false;
        }
        ret = rclc_executor_add_timer(&executor, &timer_);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_executor_add_timer:" + std::to_string(ret));
            ret = rcl_timer_fini(&timer_);
            if (ret != RCL_RET_OK) {
                serial_print("rcl_timer_fini error: " + std::to_string(ret));
            }
            return false;
        }
        timer_initialized_ = true;
    }

    if (!motion_params_service_initialized_) {
        ret = rclc_service_init_default(&motion_params_service_, &node_, ROSIDL_GET_SRV_TYPE_SUPPORT(motion_params_service, srv, MotionParamsService), "/motion_params_service");
        if (ret != RCL_RET_OK) {
            serial_print("rclc_service_init_default motion_params_service:" + std::to_string(ret));
            return false;
        }
        ret = rclc_executor_add_service(&executor, &motion_params_service_, &motion_params_request_, &motion_params_response_, motion_params_service_callback);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_executor_add_service motion_params_service:" + std::to_string(ret));
            ret = rcl_service_fini(&motion_params_service_, &node_);
            if (ret != RCL_RET_OK) {
                serial_print("rcl_service_fini motion_params_service error: " + std::to_string(ret));
            }
            return false;
        }
        motion_params_service_initialized_ = true;
    }

    return true;
}

bool MicroRos::reCreate_service_timer() {
    if (timer_initialized_) {
        auto ret = rclc_executor_remove_timer(&executor, &timer_);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_executor_remove_timer error: " + std::to_string(ret));
        }
        ret = rcl_timer_fini(&timer_);
        if (ret != RCL_RET_OK) {
            serial_print("rcl_timer_fini error: " + std::to_string(ret));
        }
        timer_initialized_ = false;
    }

    if (timer_initialized_ == false) {
        auto ret = rclc_timer_init_default(&timer_, &support_, RCL_MS_TO_NS(twoWheelDiffModel.get_milliseconds()), timer_callback);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_timer_init_default:" + std::to_string(ret));
            return false;
        }

        ret = rclc_executor_add_timer(&executor, &timer_);
        if (ret != RCL_RET_OK) {
            serial_print("rclc_executor_add_timer:" + std::to_string(ret));
            ret = rcl_timer_fini(&timer_);
            if (ret != RCL_RET_OK) {
                serial_print("rcl_timer_fini error: " + std::to_string(ret));
            }
            return false;
        }
    }
    timer_initialized_ = true;
    return true;
}

void MicroRos::start_task() {
    if (enable_task_run == false) {
        enable_task_run = true;
        xTaskCreate(microros_task, "microros_task", 8192, this, 1, NULL);
    }
}

void MicroRos::stop_task() {
    enable_task_run = false;
}

rcl_publisher_t *MicroRos::get_motion_status_publisher() {
    return &motion_status_publisher_;
}

rcl_publisher_t *MicroRos::get_odom_publisher() {
    return &odom_publisher_;
}

rcl_publisher_t *MicroRos::get_serial_msg_publisher() {
    return &serial_msg_publisher_;
}

void microros_task(void *args) {
    MicroRos *microRos = static_cast<MicroRos *>(args);
    bool init_success = false;
    microRos->connected = false;
    while (microRos->enable_task_run) {
        if (!init_success) {
            if (!microRos->init()) {
                serial_print("microros init failed, try again...");
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            } else {
                serial_print("microros task is running...");
                init_success = true;
            }
        }

        rclc_executor_spin_some(&microRos->executor, RCL_MS_TO_NS(twoWheelDiffModel.get_milliseconds()));
        if (rmw_uros_ping_agent(100, 10) != RCL_RET_OK) {
            microRos->connected = false;
            serial_print("microros is disconnected, reconnecting...");
            init_success = false;
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            microRos->connected = true;
            vTaskDelay(pdMS_TO_TICKS(twoWheelDiffModel.get_milliseconds()));
        }
    }
    microRos->clean();
    vTaskDelete(NULL);
}

void motion_params_service_callback(const void *req, void *res) {
    const motion_params_service__srv__MotionParamsService_Request *request = (const motion_params_service__srv__MotionParamsService_Request *)req;
    motion_params_service__srv__MotionParamsService_Response *response = (motion_params_service__srv__MotionParamsService_Response *)res;

    if (request->mode == ServiceType::Restart) {
        twoWheelDiffModel.restart();
    } else if (request->mode == ServiceType::Brake) {
        twoWheelDiffModel.brake();
    } else if (request->mode == ServiceType::StopMove) {
        twoWheelDiffModel.stop_move();
    } else if (request->mode == ServiceType::MoveFront) {
        twoWheelDiffModel.move_front();
    } else if (request->mode == ServiceType::MoveBack) {
        twoWheelDiffModel.move_back();
    } else if (request->mode == ServiceType::MoveLeft) {
        twoWheelDiffModel.move_left();
    } else if (request->mode == ServiceType::MoveRight) {
        twoWheelDiffModel.move_right();
    }

    else if (request->mode == ServiceType::SetSpeedPercent) {
        twoWheelDiffModel.set_speed_percent(request->speed_percent);
        response->max_v = twoWheelDiffModel.get_max_speed();
        response->speed_percent = twoWheelDiffModel.get_speed_percent();
    }

    else if (request->mode == ServiceType::SetSpeedPlanState) {
        twoWheelDiffModel.set_speed_plan_state(request->enable_speed_plan);
    }

    else if (request->mode == ServiceType::SetNoBalanceMode) {
        twoWheelDiffModel.set_motion_mode(NoBalanceMode);
    } else if (request->mode == ServiceType::SetStaticBalanceMode) {
        twoWheelDiffModel.set_motion_mode(StaticBalanceMode);
    } else if (request->mode == ServiceType::SetDynamicBalanceMode) {
        twoWheelDiffModel.set_motion_mode(DynamicBalanceMode);
    }

    else if (request->mode == ServiceType::CalibrateMPU) {
        twoWheelDiffModel.mpu6050_calibrate();
    } else if (request->mode == ServiceType::SetOffsetMPU) {
        twoWheelDiffModel.mpu6050_set_offset();
    }

    else if (request->mode == ServiceType::ReadParams) {
        twoWheelDiffModel.read_params(response);
    } else if (request->mode == ServiceType::WriteParams) {
        if (twoWheelDiffModel.get_milliseconds() != request->milliseconds && request->milliseconds > 0) {
            if (microRos.reCreate_service_timer()) {
                serial_print("定时器重新创建成功!");
            } else {
                serial_print("定时器重新创建失败.");
            }
        }
        twoWheelDiffModel.set_milliseconds(request->milliseconds);
        twoWheelDiffModel.set_loop_period_cnt(request->attitude_loop_milliseconds_cnt, request->speed_loop_milliseconds_cnt);

        twoWheelDiffModel.set_position_pid_params(request->position_p, request->position_i, request->position_d, request->position_max_integral);
        twoWheelDiffModel.set_attitude_pid_params(request->attitude_p, request->attitude_i, request->attitude_d, request->attitude_max_integral);

        twoWheelDiffModel.set_line_speed_pid_params(request->line_speed_p, request->line_speed_i, request->line_speed_d, request->line_speed_max_integral);
        twoWheelDiffModel.set_angle_speed_pid_params(request->angle_speed_p, request->angle_speed_i, request->angle_speed_d, request->angle_speed_max_integral);

        twoWheelDiffModel.set_left_motor_pid_params(request->left_motor_p, request->left_motor_i, request->left_motor_d, request->left_motor_max_integral);
        twoWheelDiffModel.set_right_motor_pid_params(request->right_motor_p, request->right_motor_i, request->right_motor_d, request->right_motor_max_integral);

        twoWheelDiffModel.set_speed_plan_parms(request->max_v, request->max_acc, request->jerk);

        response->max_v = twoWheelDiffModel.get_max_speed();
        response->speed_percent = twoWheelDiffModel.get_speed_percent();
    } else if (request->mode == ServiceType::SaveParams) {
        twoWheelDiffModel.save_params();
    }

    else if (request->mode == ServiceType::ReadSettings) {
        twoWheelDiffModel.read_settings(response);
    } else if (request->mode == ServiceType::WriteSettings) {
        twoWheelDiffModel.set_model_settings(request->wheel_diameter, request->track_width, request->pluses_per_revolution, request->revolutions_per_minute);
        twoWheelDiffModel.set_mpu6050_pins(request->pin_sda, request->pin_scl);
        twoWheelDiffModel.set_left_motor_control_pins(request->left_motor_pina, request->left_motor_pinb, request->left_encoder_pina, request->left_encoder_pinb, request->left_motor_pwm);
        twoWheelDiffModel.set_right_motor_control_pins(request->right_motor_pina, request->right_motor_pinb, request->right_encoder_pina, request->right_encoder_pinb, request->right_motor_pwm);
    } else if (request->mode == ServiceType::SaveSettings) {
        twoWheelDiffModel.save_settings();
    }

    response->state = request->mode;
    response->id = request->id;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (microRos.is_connected()) {
        rcl_ret_t ret;
        ret = rcl_publish(microRos.get_odom_publisher(), &twoWheelDiffModel.get_odom_msg(), NULL);
        if (ret != RCL_RET_OK) {
            serial_print("error: pub odom msg failed:" + std::to_string(ret));
        }

        ret = rcl_publish(microRos.get_motion_status_publisher(), &twoWheelDiffModel.get_motion_status_msg(), NULL);
        if (ret != RCL_RET_OK) {
            serial_print("error: pub motion status msg failed:" + std::to_string(ret));
        }
    }
}

void msg_twist_callback(const void *msg) {
    const geometry_msgs__msg__Twist *twist_msg = static_cast<const geometry_msgs__msg__Twist *>(msg);
    twoWheelDiffModel.start_move(twist_msg->linear.x, twist_msg->angular.z);
}

void _serial_print(const std::string &msg) {
    Serial.println(msg.c_str());

    if (microRos.is_connected()) {
        std_msgs__msg__String ros_msg;
        rosidl_runtime_c__String__init(&ros_msg.data);
        rosidl_runtime_c__String__assign(&ros_msg.data, msg.c_str());

        rcl_ret_t ret;
        ret = rcl_publish(microRos.get_serial_msg_publisher(), &ros_msg, NULL);
        if (ret != RCL_RET_OK) {
            Serial.print("error: pub serial msg failed:");
            Serial.println(ret);
        }
    }
}