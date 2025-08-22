#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(std::make_shared<Ui::MainWindow>()) {
    ui->setupUi(this);
    setWindowTitle("差速小车控制系统");

    ui->widget_main->setLayout(new QVBoxLayout());
    QSplitter *main_splitter = new QSplitter(Qt::Horizontal);

    main_splitter->addWidget(_initCustomPlot());
    main_splitter->addWidget(_initTextEdit());
    main_splitter->setCollapsible(0, false);
    ui->widget_main->layout()->addWidget(main_splitter);
    main_splitter->setSizes({1000, 0});

    connect(ui->pushButton_restart, &QPushButton::clicked, this, &MainWindow::restart);
    connect(ui->pushButton_brake, &QPushButton::clicked, this, &MainWindow::brake);
    connect(ui->pushButton_stop_move, &QPushButton::clicked, this, &MainWindow::stop_move);
    connect(ui->pushButton_set_speed, &QPushButton::clicked, this, &MainWindow::set_speed);
    connect(ui->pushButton_move_front, &QPushButton::clicked, this, &MainWindow::move_front);
    connect(ui->pushButton_move_back, &QPushButton::clicked, this, &MainWindow::move_back);
    connect(ui->pushButton_move_left, &QPushButton::clicked, this, &MainWindow::move_left);
    connect(ui->pushButton_move_right, &QPushButton::clicked, this, &MainWindow::move_right);

    connect(ui->pushButton_write_params, &QPushButton::clicked, this, &MainWindow::on_write_params);
    connect(ui->pushButton_read_params, &QPushButton::clicked, this, &MainWindow::on_read_params);
    connect(ui->pushButton_save_params, &QPushButton::clicked, this, &MainWindow::on_save_params);

    connect(ui->pushButton_write_settings, &QPushButton::clicked, this, &MainWindow::on_write_settings);
    connect(ui->pushButton_read_settings, &QPushButton::clicked, this, &MainWindow::on_read_settings);
    connect(ui->pushButton_save_settings, &QPushButton::clicked, this, &MainWindow::on_save_settings);

    connect(ui->pushButton_path_move, &QPushButton::clicked, this, &MainWindow::start_move_path);
    connect(ui->pushButton_path_stop, &QPushButton::clicked, this, &MainWindow::stop_move_path);
    connect(ui->pushButton_clear_plot, &QPushButton::clicked, this, &MainWindow::clear_plot);
    connect(ui->radioButton_polar_coordinates, &QRadioButton::clicked, this, &MainWindow::on_change_path_mode);
    connect(ui->radioButton_cartesian_coordinates, &QCheckBox::clicked, this, &MainWindow::on_change_path_mode);
    connect(ui->pushButton_calibrate_imu, &QPushButton::clicked, this, &MainWindow::on_calibrate_imu);
    connect(ui->pushButton_setOffset, &QPushButton::clicked, this, &MainWindow::on_set_offset_imu);
    connect(ui->pushButton_clear_cmd_info, &QPushButton::clicked, this, [this]() { ui->textEdit_cmd->clear(); });
    connect(ui->pushButton_show_dockWidget, &QPushButton::clicked, this, [this]() { ui->dockWidget_set->show(); });

    connect(ui->radioButton_dynamic_balance, &QRadioButton::clicked, this, &MainWindow::set_motion_mode);
    connect(ui->radioButton_static_balance, &QRadioButton::clicked, this, &MainWindow::set_motion_mode);
    connect(ui->radioButton_no_balance, &QRadioButton::clicked, this, &MainWindow::set_motion_mode);

    connect(ui->horizontalSlider_speed_percent, &QSlider::sliderReleased, this, &MainWindow::set_speed_percent);

    connect(ui->checkBox_enable_speed_plan, &QCheckBox::stateChanged, this, &MainWindow::set_speed_plan_state);

    ui->checkBox_graph_0->click();
    ui->checkBox_graph_1->click();
    ui->checkBox_graph_2->click();
    ui->checkBox_graph_3->click();
    connect(ui->checkBox_graph_0, &QCheckBox::clicked, this, &MainWindow::on_graph_visible_changed);
    connect(ui->checkBox_graph_1, &QCheckBox::clicked, this, &MainWindow::on_graph_visible_changed);
    connect(ui->checkBox_graph_2, &QCheckBox::clicked, this, &MainWindow::on_graph_visible_changed);
    connect(ui->checkBox_graph_3, &QCheckBox::clicked, this, &MainWindow::on_graph_visible_changed);

    _initStatusBar();
    _initGamepad();
    _initNodeThread();
    _initTimer();

    // 初始化
    ui->radioButton_cartesian_coordinates->click();

    on_update_status();
    _update_imu_label();
    _update_odom_label();
    on_graph_visible_changed();
    ui->tabWidget_motion->setCurrentIndex(0);
    ui->tabWidget_settings->setCurrentIndex(0);
    ui->dockWidget_set->close();

    // _readConfigJson();
    nodeThread_->start();
    status_timer_->start();
}

MainWindow::~MainWindow() {
    agent_process_->terminate();
    if (!agent_process_->waitForFinished(3000)) { // 等待 3 秒
        agent_process_->kill();                   // 强制终止
        agent_process_->waitForFinished();        // 再等它退出
    }

    QProcess process;
    process.start("pgrep", QStringList() << "-f" << "micro_ros_agent");
    process.waitForFinished();
    QString output = process.readAllStandardOutput();
    QStringList pids = output.split("\n", Qt::SkipEmptyParts);
    for (const QString &pid : pids) {
        QProcess::execute("kill", QStringList() << "-9" << pid);
    }
}

void MainWindow::_initTimer() {
    status_timer_ = std::make_shared<QTimer>();
    status_timer_->setInterval(500);
    connect(status_timer_.get(), &QTimer::timeout, this, &MainWindow::on_update_status);

    cmd_status_timer_ = std::make_shared<QTimer>();
    cmd_status_timer_->setInterval(5000);
    cmd_status_timer_->setSingleShot(true);
    connect(cmd_status_timer_.get(), &QTimer::timeout, this, [this]() { cmd_status_label_->clear(); cmd_status_label_->setStyleSheet(""); });
}

void MainWindow::_initNodeThread() {
    nodeThread_ = std::make_shared<NodeQThread>("node_thread");
    connect(nodeThread_.get(), &NodeQThread::motionStatusMsgChanged, this, &MainWindow::on_recv_motion_status_msg);
    connect(nodeThread_.get(), &NodeQThread::motionParamsServiceResponsed, this, &MainWindow::on_recv_motion_params_service_response);
    connect(nodeThread_.get(), &NodeQThread::odomMsgChanged, this, &MainWindow::on_recv_odom_msg);
    connect(nodeThread_.get(), &NodeQThread::serialMsgChanged, this, &MainWindow::on_recv_serial_msg);

    connect(nodeThread_.get(), &NodeQThread::microRosConnected, this, [this]() { on_read_params();on_read_settings();clear_plot(); });
    connect(nodeThread_.get(), &NodeQThread::commandStateChanged, this, &MainWindow::on_command_state_changed);
}

QSplitter *MainWindow::_initCustomPlot() {
    QSplitter *customPlot_splitter = new QSplitter(Qt::Vertical);
    wheel_speed_customPlot_ = std::make_shared<QCustomPlot>();
    merge_speed_customPlot_ = std::make_shared<QCustomPlot>();
    customPlot_splitter->addWidget(wheel_speed_customPlot_.get());
    customPlot_splitter->addWidget(merge_speed_customPlot_.get());
    customPlot_splitter->setCollapsible(0, false);
    customPlot_splitter->setCollapsible(1, false);

    wheel_speed_customPlot_->addGraph();
    wheel_speed_customPlot_->addGraph();
    wheel_speed_customPlot_->addGraph();
    wheel_speed_customPlot_->addGraph();
    wheel_speed_customPlot_->setWindowTitle("速度曲线");
    wheel_speed_customPlot_->xAxis->setLabel("时间 (s)");
    wheel_speed_customPlot_->xAxis->setNumberPrecision(3);
    wheel_speed_customPlot_->yAxis->setLabel("速度 (m/s)");
    wheel_speed_customPlot_->yAxis->setNumberPrecision(3);
    wheel_speed_customPlot_->graph(0)->setPen(QPen(Qt::red));
    wheel_speed_customPlot_->graph(0)->setName("左轮实际速度");
    wheel_speed_customPlot_->graph(1)->setPen(QPen(Qt::green));
    wheel_speed_customPlot_->graph(1)->setName("左轮目标速度");
    wheel_speed_customPlot_->graph(2)->setPen(QPen(Qt::blue));
    wheel_speed_customPlot_->graph(2)->setName("右轮实际速度");
    wheel_speed_customPlot_->graph(3)->setPen(QPen(Qt::darkYellow));
    wheel_speed_customPlot_->graph(3)->setName("右轮目标速度");
    wheel_speed_customPlot_->legend->setVisible(true);
    wheel_speed_customPlot_->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignLeft);
    wheel_speed_customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iMultiSelect | QCP::iSelectPlottables | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectItems | QCP::iSelectOther | QCP::iSelectPlottablesBeyondAxisRect);

    merge_speed_customPlot_->addGraph();
    merge_speed_customPlot_->addGraph();
    merge_speed_customPlot_->addGraph();
    merge_speed_customPlot_->addGraph();
    merge_speed_customPlot_->setWindowTitle("合成速度曲线");
    merge_speed_customPlot_->xAxis->setNumberPrecision(3);
    merge_speed_customPlot_->xAxis->setLabel("时间 (s)");
    merge_speed_customPlot_->yAxis->setLabel("速度 (m/s^2)");
    merge_speed_customPlot_->yAxis->setNumberPrecision(3);
    merge_speed_customPlot_->graph(0)->setPen(QPen(Qt::red));
    merge_speed_customPlot_->graph(0)->setName("合成实际线速度");
    merge_speed_customPlot_->graph(1)->setPen(QPen(Qt::green));
    merge_speed_customPlot_->graph(1)->setName("合成目标线速度");
    merge_speed_customPlot_->graph(2)->setPen(QPen(Qt::blue));
    merge_speed_customPlot_->graph(2)->setName("合成实际角速度");
    merge_speed_customPlot_->graph(3)->setPen(QPen(Qt::darkYellow));
    merge_speed_customPlot_->graph(3)->setName("合成目标角速度");
    merge_speed_customPlot_->legend->setVisible(true);
    merge_speed_customPlot_->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignLeft);
    merge_speed_customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iMultiSelect | QCP::iSelectPlottables | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectItems | QCP::iSelectOther | QCP::iSelectPlottablesBeyondAxisRect);

    return customPlot_splitter;
}

QSplitter *MainWindow::_initTextEdit() {
    QSplitter *info_textEdit_splitter = new QSplitter(Qt::Vertical);

    motion_info_text_ = std::make_shared<QTextEdit>();
    motion_info_text_->setReadOnly(true);

    agent_process_ = std::make_shared<QProcess>();
    agent_info_textEdit_ = std::make_shared<QTextEdit>();
    agent_info_textEdit_->setReadOnly(true);

    auto options = QRegularExpression("\\x1B\\[[0-9;]*[mK]");

    connect(agent_process_.get(), &QProcess::readyReadStandardOutput, this, [this, options]() {
        QString output = agent_process_->readAllStandardOutput();
        QString html = QString("<span style='color:black;'>%1</span>").arg(output.remove(options).toHtmlEscaped());
        agent_info_textEdit_->append(html);
    });
    connect(agent_process_.get(), &QProcess::readyReadStandardError, this, [this, options]() {
        QString error = agent_process_->readAllStandardError();
        QString html = QString("<span style='color:red;'>%1</span>").arg(error.remove(options).toHtmlEscaped());
        agent_info_textEdit_->append(html);
    });

    info_textEdit_splitter->addWidget(motion_info_text_.get());
    info_textEdit_splitter->addWidget(agent_info_textEdit_.get());

    agent_process_->start("ros2", QStringList() << "run" << "micro_ros_agent" << "micro_ros_agent" << "udp4" << "--port" << "8888");

    return info_textEdit_splitter;
}
void MainWindow::_initStatusBar() {
    micro_ros_status_label_ = std::make_shared<QLabel>();
    gamepad_status_label_ = std::make_shared<QLabel>();
    cmd_status_label_ = std::make_shared<QLabel>();
    cmd_status_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    ui->statusBar->addWidget(micro_ros_status_label_.get());
    ui->statusBar->addWidget(gamepad_status_label_.get());
    ui->statusBar->addPermanentWidget(cmd_status_label_.get(), 1);
}

void MainWindow::_initGamepad() {
    gamepad_ = std::make_shared<QGamepad>(0);
    connect(gamepad_.get(), &QGamepad::axisLeftXChanged, this, [this](double) {
        std::shared_ptr<geometry_msgs::msg::Twist> twist = std::make_shared<geometry_msgs::msg::Twist>();
        twist->linear.x = gamepad_->axisLeftY() * -0.4;
        twist->angular.z = gamepad_->axisLeftX() * -4.5;
        auto cmd_string = _get_cmd_string_prefix() + "手柄遥感指令";
        _publish_twist(twist, cmd_string);
    });

    connect(gamepad_.get(), &QGamepad::axisLeftXChanged, this, [this](double) {
        std::shared_ptr<geometry_msgs::msg::Twist> twist = std::make_shared<geometry_msgs::msg::Twist>();
        twist->linear.x = gamepad_->axisLeftY() * -0.4;
        twist->angular.z = gamepad_->axisLeftX() * -4.5;
        auto cmd_string = _get_cmd_string_prefix() + "手柄遥感指令";
        _publish_twist(twist, cmd_string);
    });

    connect(gamepad_.get(), &QGamepad::buttonGuideChanged, this, [this](bool) {
        ui->pushButton_stop_move->click();
    });

    connect(gamepad_.get(), &QGamepad::buttonYChanged, this, [this](bool value) {
        if (value) {
            ui->pushButton_move_front->click();
        } else {
            ui->pushButton_stop_move->click();
        }
    });
    connect(gamepad_.get(), &QGamepad::buttonAChanged, this, [this](bool value) {
        if (value) {
            ui->pushButton_move_back->click();
        } else {
            ui->pushButton_stop_move->click();
        }
    });
    connect(gamepad_.get(), &QGamepad::buttonXChanged, this, [this](bool value) {
        if (value) {
            ui->pushButton_move_left->click();
        } else {
            ui->pushButton_stop_move->click();
        }
    });
    connect(gamepad_.get(), &QGamepad::buttonBChanged, this, [this](bool value) {
        if (value) {
            ui->pushButton_move_right->click();
        } else {
            ui->pushButton_stop_move->click();
        }
    });
}

QString MainWindow::_get_cmd_string_prefix() {
    return " [" + QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz") + "] ";
}

void MainWindow::set_speed() {
    std::shared_ptr<geometry_msgs::msg::Twist> twist = std::make_shared<geometry_msgs::msg::Twist>();
    twist->linear.x = ui->doubleSpinBox_lineX_spd->value();
    twist->linear.y = ui->doubleSpinBox_lineY_spd->value();
    twist->linear.z = ui->doubleSpinBox_lineZ_spd->value();
    twist->angular.x = ui->doubleSpinBox_angleX_spd->value();
    twist->angular.y = ui->doubleSpinBox_angleY_spd->value();
    twist->angular.z = ui->doubleSpinBox_angleZ_spd->value();
    auto cmd_string = _get_cmd_string_prefix() + "设置速度指令";
    _publish_twist(twist, cmd_string);
}

void MainWindow::_publish_twist(std::shared_ptr<geometry_msgs::msg::Twist> twist, QString &cmd_string) {
    auto id = QDateTime::currentMSecsSinceEpoch();
    QThread::msleep(1);
    commpand_map_.insert(id, cmd_string);
    if (nodeThread_->add_twist(id, twist)) {
        on_command_state_changed(id, CommandState::Add);
    } else {
        on_command_state_changed(id, CommandState::Fail);
    };
}

void MainWindow::restart() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::Restart;
    auto cmd_string = _get_cmd_string_prefix() + "重启指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::brake() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::Brake;
    auto cmd_string = _get_cmd_string_prefix() + "刹车指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::stop_move() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::StopMove;
    auto cmd_string = _get_cmd_string_prefix() + "停止指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::move_front() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::MoveFront;
    auto cmd_string = _get_cmd_string_prefix() + "前进指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::move_back() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::MoveBack;
    auto cmd_string = _get_cmd_string_prefix() + "后退指令";
    _ask_motion_params_service(request, cmd_string);
}
void MainWindow::move_left() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::MoveLeft;
    auto cmd_string = _get_cmd_string_prefix() + "左转指令";
    _ask_motion_params_service(request, cmd_string);
}
void MainWindow::move_right() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::MoveRight;
    auto cmd_string = _get_cmd_string_prefix() + "右转指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::start_move_path() {
    // motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    // request->mode = 1004;
    // // request->can_run = true;
    // // request->distance = ui->doubleSpinBox_path_dis->value();
    // // request->angle = ui->doubleSpinBox_path_angle->value();
    // auto cmd_string = _get_cmd_string_prefix() + "开始路径规划指令";
    // _ask_motion_params_service(request, cmd_string);
}

void MainWindow::stop_move_path() {
    // motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    // request->mode = 1005;
    // // request->can_run = false;
    // auto cmd_string = _get_cmd_string_prefix() + "停止路径规划指令";
    // _ask_motion_params_service(request, cmd_string);
}

void MainWindow::set_motion_mode() {
    int mode = ServiceType::SetNoBalanceMode;
    QString cmd_string = _get_cmd_string_prefix();
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    if (ui->radioButton_dynamic_balance->isChecked()) {
        mode = ServiceType::SetDynamicBalanceMode;
        cmd_string += "设置动态平衡指令";
    } else if (ui->radioButton_static_balance->isChecked()) {
        mode = ServiceType::SetStaticBalanceMode;
        cmd_string += "设置静态平衡指令";
    } else if (ui->radioButton_no_balance->isChecked()) {
        mode = ServiceType::SetNoBalanceMode;
        cmd_string += "取消平衡指令";
    }
    request->mode = mode;
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_calibrate_imu() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::CalibrateMPU;
    auto cmd_string = _get_cmd_string_prefix() + "IMU标定指令";
    _ask_motion_params_service(request, cmd_string);
    on_read_params();
}

void MainWindow::on_set_offset_imu() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::SetOffsetMPU;
    auto cmd_string = _get_cmd_string_prefix() + "IMU设置偏移指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::set_speed_percent() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::SetSpeedPercent;
    request->speed_percent = ui->horizontalSlider_speed_percent->value() * 1.0 / ui->horizontalSlider_speed_percent->maximum();
    _update_speed_percent_label(request->speed_percent);
    auto cmd_string = _get_cmd_string_prefix() + "设置速度指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::set_speed_plan_state() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::SetSpeedPlanState;
    request->enable_speed_plan = ui->checkBox_enable_speed_plan->isChecked();
    auto cmd_string = _get_cmd_string_prefix() + "设置速度规划指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_read_params() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::ReadParams;
    auto cmd_string = _get_cmd_string_prefix() + "读取参数指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_write_params() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    if (ui->doubleSpinBox_max_v->value() <= 0) {
        QMessageBox::critical(this, "错误", "最大速度参数不能为负数或零，写入失败！");
        return;
    }
    if (ui->doubleSpinBox_max_acc->value() <= 0) {
        QMessageBox::critical(this, "错误", "最大加速度参数不能为负数或零，写入失败！");
        return;
    }
    if (ui->doubleSpinBox_jerk->value() <= 0) {
        QMessageBox::critical(this, "错误", "加加速度参数不能为负数或零，写入失败！");
        return;
    }
    if (ui->spinBox_millseconds->value() <= 0) {
        QMessageBox::critical(this, "错误", "时间周期参数不能为负数或零，写入失败！");
        return;
    }
    request->mode = ServiceType::WriteParams;

    request->position_p = ui->doubleSpinBox_position_p->value();
    request->position_i = ui->doubleSpinBox_position_i->value();
    request->position_d = ui->doubleSpinBox_position_d->value();
    request->position_max_integral = ui->doubleSpinBox_position_max_i->value();

    request->attitude_p = ui->doubleSpinBox_attitude_p->value();
    request->attitude_i = ui->doubleSpinBox_attitude_i->value();
    request->attitude_d = ui->doubleSpinBox_attitude_d->value();
    request->attitude_max_integral = ui->doubleSpinBox_attitude_max_i->value();

    request->line_speed_p = ui->doubleSpinBox_line_speed_p->value();
    request->line_speed_i = ui->doubleSpinBox_line_speed_i->value();
    request->line_speed_d = ui->doubleSpinBox_line_speed_d->value();
    request->line_speed_max_integral = ui->doubleSpinBox_line_speed_max_i->value();

    request->angle_speed_p = ui->doubleSpinBox_angle_speed_p->value();
    request->angle_speed_i = ui->doubleSpinBox_angle_speed_i->value();
    request->angle_speed_d = ui->doubleSpinBox_angle_speed_d->value();
    request->angle_speed_max_integral = ui->doubleSpinBox_angle_speed_max_i->value();

    request->left_motor_p = ui->doubleSpinBox_left_motor_p->value();
    request->left_motor_i = ui->doubleSpinBox_left_motor_i->value();
    request->left_motor_d = ui->doubleSpinBox_left_motor_d->value();
    request->left_motor_max_integral = ui->doubleSpinBox_left_motor_max_i->value();

    request->right_motor_p = ui->doubleSpinBox_right_motor_p->value();
    request->right_motor_i = ui->doubleSpinBox_right_motor_i->value();
    request->right_motor_d = ui->doubleSpinBox_right_motor_d->value();
    request->right_motor_max_integral = ui->doubleSpinBox_right_motor_max_i->value();

    request->max_v = ui->doubleSpinBox_max_v->value();
    request->max_acc = ui->doubleSpinBox_max_acc->value();
    request->jerk = ui->doubleSpinBox_jerk->value();

    request->milliseconds = ui->spinBox_millseconds->value();
    request->attitude_loop_milliseconds_cnt = ui->spinBox_attitude_loop_milliseconds_cnt->value();
    request->speed_loop_milliseconds_cnt = ui->spinBox_speed_loop_milliseconds_cnt->value();

    auto cmd_string = _get_cmd_string_prefix() + "写入参数指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_save_params() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::SaveParams;
    auto cmd_string = _get_cmd_string_prefix() + "保存参数指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_read_settings() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::ReadSettings;
    auto cmd_string = _get_cmd_string_prefix() + "读取配置指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_write_settings() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::WriteSettings;
    request->wheel_diameter = ui->doubleSpinBox_wheel_diameter->value();
    request->track_width = ui->doubleSpinBox_track_width->value();
    request->pluses_per_revolution = ui->spinBox_pluses_per_revolution->value();
    request->revolutions_per_minute = ui->spinBox_revolutions_per_minute->value();

    request->pin_sda = ui->spinBox_MPU6050_pin0->value();
    request->pin_scl = ui->spinBox_MPU6050_pin1->value();

    request->left_motor_pina = ui->spinBox_motor0_pin0->value();
    request->left_motor_pinb = ui->spinBox_motor0_pin1->value();
    request->left_motor_pwm = ui->spinBox_motor0_PWMPin->value();
    request->left_encoder_pina = ui->spinBox_encoder0_pin0->value();
    request->left_encoder_pinb = ui->spinBox_encoder0_pin1->value();

    request->right_motor_pina = ui->spinBox_motor1_pin0->value();
    request->right_motor_pinb = ui->spinBox_motor1_pin1->value();
    request->right_motor_pwm = ui->spinBox_motor1_PWMpin->value();
    request->right_encoder_pina = ui->spinBox_encoder1_pin0->value();
    request->right_encoder_pinb = ui->spinBox_encoder1_pin1->value();

    auto cmd_string = _get_cmd_string_prefix() + "写入配置指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::on_save_settings() {
    motion_params_service::srv::MotionParamsService::Request::SharedPtr request(std::make_shared<motion_params_service::srv::MotionParamsService::Request>());
    request->mode = ServiceType::SaveSettings;
    auto cmd_string = _get_cmd_string_prefix() + "保存配置指令";
    _ask_motion_params_service(request, cmd_string);
}

void MainWindow::_ask_motion_params_service(motion_params_service::srv::MotionParamsService::Request::SharedPtr request, QString &cmd_string) {
    QThread::msleep(1);
    auto id = QDateTime::currentMSecsSinceEpoch();
    commpand_map_.insert(id, cmd_string);
    request->id = id;
    if (nodeThread_->add_motion_params_service(id, request)) {
        on_command_state_changed(id, CommandState::Add);
    } else {
        on_command_state_changed(id, CommandState::Fail);
    }
}

void MainWindow::on_recv_motion_params_service_response(motion_params_service::srv::MotionParamsService::Response::SharedPtr response) {
    on_command_state_changed(response->id, CommandState::Success);

    if (response->state == ServiceType::ReadParams) {
        ui->spinBox_millseconds->setValue(response->milliseconds);
        ui->spinBox_attitude_loop_milliseconds_cnt->setValue(response->attitude_loop_milliseconds_cnt);
        ui->spinBox_speed_loop_milliseconds_cnt->setValue(response->speed_loop_milliseconds_cnt);

        ui->checkBox_enable_speed_plan->blockSignals(true);
        ui->checkBox_enable_speed_plan->setChecked(response->enable_speed_plan);
        ui->checkBox_enable_speed_plan->blockSignals(false);

        ui->radioButton_no_balance->blockSignals(true);
        ui->radioButton_dynamic_balance->blockSignals(true);
        ui->radioButton_static_balance->blockSignals(true);
        if (response->motion_mode == MotionMode::NoBalanceMode)
            ui->radioButton_no_balance->click();
        else if (response->motion_mode == MotionMode::StaticBalanceMode)
            ui->radioButton_static_balance->click();
        else if (response->motion_mode == MotionMode::DynamicBalanceMode)
            ui->radioButton_dynamic_balance->click();
        ui->radioButton_no_balance->blockSignals(false);
        ui->radioButton_dynamic_balance->blockSignals(false);
        ui->radioButton_static_balance->blockSignals(false);

        ui->horizontalSlider_speed_percent->blockSignals(true);
        ui->horizontalSlider_speed_percent->setValue(response->speed_percent * ui->horizontalSlider_speed_percent->maximum());
        ui->horizontalSlider_speed_percent->blockSignals(false);
        _update_speed_percent_label(response->speed_percent);

        ui->doubleSpinBox_max_v->setValue(response->max_v);
        ui->doubleSpinBox_max_acc->setValue(response->max_acc);
        ui->doubleSpinBox_jerk->setValue(response->jerk);

        ui->doubleSpinBox_position_p->setValue(response->position_p);
        ui->doubleSpinBox_position_i->setValue(response->position_i);
        ui->doubleSpinBox_position_d->setValue(response->position_d);
        ui->doubleSpinBox_position_max_i->setValue(response->position_max_integral);

        ui->doubleSpinBox_attitude_p->setValue(response->attitude_p);
        ui->doubleSpinBox_attitude_i->setValue(response->attitude_i);
        ui->doubleSpinBox_attitude_d->setValue(response->attitude_d);
        ui->doubleSpinBox_attitude_max_i->setValue(response->attitude_max_integral);

        ui->doubleSpinBox_line_speed_p->setValue(response->line_speed_p);
        ui->doubleSpinBox_line_speed_i->setValue(response->line_speed_i);
        ui->doubleSpinBox_line_speed_d->setValue(response->line_speed_d);
        ui->doubleSpinBox_line_speed_max_i->setValue(response->line_speed_max_integral);

        ui->doubleSpinBox_angle_speed_p->setValue(response->angle_speed_p);
        ui->doubleSpinBox_angle_speed_i->setValue(response->angle_speed_i);
        ui->doubleSpinBox_angle_speed_d->setValue(response->angle_speed_d);
        ui->doubleSpinBox_angle_speed_max_i->setValue(response->angle_speed_max_integral);

        ui->doubleSpinBox_left_motor_p->setValue(response->left_motor_p);
        ui->doubleSpinBox_left_motor_i->setValue(response->left_motor_i);
        ui->doubleSpinBox_left_motor_d->setValue(response->left_motor_d);
        ui->doubleSpinBox_left_motor_max_i->setValue(response->left_motor_max_integral);

        ui->doubleSpinBox_right_motor_p->setValue(response->right_motor_p);
        ui->doubleSpinBox_right_motor_i->setValue(response->right_motor_i);
        ui->doubleSpinBox_right_motor_d->setValue(response->right_motor_d);
        ui->doubleSpinBox_right_motor_max_i->setValue(response->right_motor_max_integral);

        ui->spinBox_accel_offset_x->setValue(response->accel_offset_x);
        ui->spinBox_accel_offset_y->setValue(response->accel_offset_y);
        ui->spinBox_accel_offset_z->setValue(response->accel_offset_z);
        ui->spinBox_gyro_offset_x->setValue(response->gyro_offset_x);
        ui->spinBox_gyro_offset_y->setValue(response->gyro_offset_y);
        ui->spinBox_gyro_offset_z->setValue(response->gyro_offset_z);
    } else if (response->state == ServiceType::WriteParams || response->state == ServiceType::SetSpeedPercent) {
        ui->horizontalSlider_speed_percent->blockSignals(true);
        ui->horizontalSlider_speed_percent->setValue(response->speed_percent * ui->horizontalSlider_speed_percent->maximum());
        ui->horizontalSlider_speed_percent->blockSignals(false);
        ui->doubleSpinBox_max_v->setValue(response->max_v);
        _update_speed_percent_label(response->speed_percent);
    } else if (response->state == ServiceType::ReadSettings) {
        ui->doubleSpinBox_wheel_diameter->setValue(response->wheel_diameter);
        ui->doubleSpinBox_track_width->setValue(response->track_width);
        ui->spinBox_pluses_per_revolution->setValue(response->pluses_per_revolution);
        ui->spinBox_revolutions_per_minute->setValue(response->revolutions_per_minute);

        ui->spinBox_MPU6050_pin0->setValue(response->pin_sda);
        ui->spinBox_MPU6050_pin1->setValue(response->pin_scl);

        ui->spinBox_motor0_pin0->setValue(response->left_motor_pina);
        ui->spinBox_motor0_pin1->setValue(response->left_motor_pinb);
        ui->spinBox_motor0_PWMPin->setValue(response->left_motor_pwm);
        ui->spinBox_encoder0_pin0->setValue(response->left_encoder_pina);
        ui->spinBox_encoder0_pin1->setValue(response->left_encoder_pinb);

        ui->spinBox_motor1_pin0->setValue(response->right_motor_pina);
        ui->spinBox_motor1_pin1->setValue(response->right_motor_pinb);
        ui->spinBox_motor1_PWMpin->setValue(response->right_motor_pwm);
        ui->spinBox_encoder1_pin0->setValue(response->right_encoder_pina);
        ui->spinBox_encoder1_pin1->setValue(response->right_encoder_pinb);
    }
}

void MainWindow::on_command_state_changed(int64_t id, int state) {
    auto cmd_string = commpand_map_.value(id, QString());
    if (!cmd_string.isEmpty()) {
        _update_cmd_status_label(cmd_string, state);
        if (state == CommandState::Fail || state == CommandState::Success) {
            commpand_map_.remove(id);
        }
    }
}

void MainWindow::on_update_status() {
    static const QString OK_STYLESHEET = "color:green;font-size:20px;"; // background-color:green;
    static const QString ERROR_STYLESHEET = "color:red;font-size:20px;";

    if (nodeThread_->micro_ros_is_online()) {
        micro_ros_status_label_->setText(" micro ros 已连接");
        micro_ros_status_label_->setStyleSheet(OK_STYLESHEET);
    } else {
        micro_ros_status_label_->setText(" micro ros 已断开");
        micro_ros_status_label_->setStyleSheet(ERROR_STYLESHEET);
    }

    if (!gamepad_->isConnected()) {
        gamepad_status_label_->setText(" 手柄未连接");
        gamepad_status_label_->setStyleSheet(ERROR_STYLESHEET);
        QList<int> gamepadIds = QGamepadManager::instance()->connectedGamepads();
        if (gamepadIds.size() > 0) {
            gamepad_->setDeviceId(gamepadIds[0]);
        }
    } else {
        gamepad_status_label_->setText(" 手柄已连接");
        gamepad_status_label_->setStyleSheet(OK_STYLESHEET);
    }
}

void MainWindow::on_recv_motion_status_msg(const motion_status_msgs::msg::MotionStatus::SharedPtr msg) {
    _update_imu_label(msg->yaw, msg->pitch, msg->roll, msg->gyro_x, msg->gyro_y, msg->gyro_z);

    if (ui->checkBox_dynamic_refresh->isChecked()) {
        double minValue = pow(10, -6);
        if (qFabs(msg->left_current_v) > minValue || qFabs(msg->left_target_v) > minValue || qFabs(msg->right_current_v) > minValue || qFabs(msg->right_target_v) > minValue || qFabs(msg->merge_current_v) > minValue || qFabs(msg->merge_target_v) > minValue || qFabs(msg->merge_current_w) > minValue || qFabs(msg->merge_target_w) > minValue) {
            if (stop_plot_) {
                stop_plot_ = false;
            }
        }

        if (stop_plot_) {
            return;
        }

        if (wheel_speed_customPlot_->graph(0)->dataCount() >= 10000) {
            clear_plot();
        }

        reference_sconds += ui->spinBox_millseconds->value() / 1000.0;
        wheel_speed_customPlot_->graph(0)->addData(reference_sconds, msg->left_current_v);
        wheel_speed_customPlot_->graph(1)->addData(reference_sconds, msg->left_target_v);
        wheel_speed_customPlot_->graph(2)->addData(reference_sconds, msg->right_current_v);
        wheel_speed_customPlot_->graph(3)->addData(reference_sconds, msg->right_target_v);

        merge_speed_customPlot_->graph(0)->addData(reference_sconds, msg->merge_current_v);
        merge_speed_customPlot_->graph(1)->addData(reference_sconds, msg->merge_target_v);
        merge_speed_customPlot_->graph(2)->addData(reference_sconds, msg->merge_current_w);
        merge_speed_customPlot_->graph(3)->addData(reference_sconds, msg->merge_target_w);

        if (ui->checkBox_dynami_follow->isChecked()) {
            merge_speed_customPlot_->rescaleAxes(); // 自动缩放坐标轴，包含所有数据
            wheel_speed_customPlot_->rescaleAxes(); // 自动缩放坐标轴，包含所有数据
        }

        wheel_speed_customPlot_->replot(); // 刷新绘图
        merge_speed_customPlot_->replot(); // 刷新绘图

        QString line = QString("t= %1\n"
                               "left_current_v= %2,  left_target_v= %3,  right_current_v= %6,  right_target_v= %7\n"
                               "merge_current_v= %4,  merge_target_v= %5,  merge_current_w= %8,  merge_target_w= %9\n")
                           .arg(reference_sconds, 0, 'f', 3)
                           .arg(msg->left_current_v, 0, 'f', 3)
                           .arg(msg->left_target_v, 0, 'f', 3)
                           .arg(msg->right_current_v, 0, 'f', 3)
                           .arg(msg->right_target_v, 0, 'f', 3)
                           .arg(msg->merge_current_v, 0, 'f', 3)
                           .arg(msg->merge_target_v, 0, 'f', 3)
                           .arg(msg->merge_current_w, 0, 'f', 3)
                           .arg(msg->merge_target_w, 0, 'f', 3);
        motion_info_text_->append(line);

        if (qFabs(msg->left_current_v) < minValue && qFabs(msg->left_target_v) < minValue && qFabs(msg->right_current_v) < minValue && qFabs(msg->right_target_v) < minValue && qFabs(msg->merge_current_v) < minValue && qFabs(msg->merge_target_v) < minValue && qFabs(msg->merge_current_w) < minValue && qFabs(msg->merge_target_w) < minValue) {
            stop_plot_ = true;
        }
        ui->groupBox_plot->setTitle(QString(("绘图设置（%1个点）")).arg(wheel_speed_customPlot_->graph(0)->dataCount()));
    }
}

void MainWindow::on_recv_odom_msg(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(tf_q);
    double roll = 0, pitch = 0, yaw = 0;
    mat.getRPY(roll, pitch, yaw);
    _update_odom_label(msg->twist.twist.linear.x, msg->twist.twist.angular.z, msg->pose.pose.position.x, msg->pose.pose.position.y, yaw * 180.0 / M_PI + 90);
}

void MainWindow::on_recv_serial_msg(const std_msgs::msg::String::SharedPtr msg) {
    QString serial_msg = QString::fromStdString(msg->data);
    _update_cmd_status_label(serial_msg, CommandState::SerialMsg);
}

void MainWindow::on_change_path_mode() {
    if (ui->radioButton_cartesian_coordinates->isChecked()) {
        ui->widget_cartesian_coordinates->setVisible(true);
        ui->widget_polar_coordinates->setVisible(false);
    } else if (ui->radioButton_polar_coordinates->isChecked()) {
        ui->widget_cartesian_coordinates->setVisible(false);
        ui->widget_polar_coordinates->setVisible(true);
    }
}

void MainWindow::on_graph_visible_changed() {
    QList<QCheckBox *> checkbox_list = {ui->checkBox_graph_0, ui->checkBox_graph_1, ui->checkBox_graph_2, ui->checkBox_graph_3};
    for (int i = 0; i < checkbox_list.size(); i++) {
        wheel_speed_customPlot_->graph(i)->setVisible(checkbox_list[i]->isChecked());
        merge_speed_customPlot_->graph(i)->setVisible(checkbox_list[i]->isChecked());
    }
    wheel_speed_customPlot_->replot();
    merge_speed_customPlot_->replot();
}

void MainWindow::clear_plot() {
    for (int i = 0; i < wheel_speed_customPlot_->graphCount(); i++) {
        wheel_speed_customPlot_->graph(i)->data()->clear();
    }

    for (int i = 0; i < merge_speed_customPlot_->graphCount(); i++) {
        merge_speed_customPlot_->graph(i)->data()->clear();
    }
    reference_sconds = 0;
    wheel_speed_customPlot_->rescaleAxes(); // 自动缩放坐标轴，包含所有数据
    wheel_speed_customPlot_->replot();      // 刷新绘图

    merge_speed_customPlot_->rescaleAxes(); // 自动缩放坐标轴，包含所有数据
    merge_speed_customPlot_->replot();      // 刷新绘图

    motion_info_text_->clear();

    ui->groupBox_plot->setTitle(QString(("绘图设置（%1个点）")).arg(wheel_speed_customPlot_->graph(0)->dataCount()));
}

void MainWindow::_update_imu_label(double yaw, double pitch, double roll, double gyro_x, double gyro_y, double gyro_z) {
    QString text = QString(
                       "姿态角:\n"
                       "    偏航角 : %1 °\n"
                       "    俯仰角 : %2 °\n"
                       "    横滚角 : %3 °\n"
                       "角速度:\n"
                       "    X轴 : %4 °/s\n"
                       "    Y轴 : %5 °/s\n"
                       "    Z轴 : %6 °/s")
                       .arg(yaw, 9, 'f', 3)
                       .arg(pitch, 9, 'f', 3)
                       .arg(roll, 9, 'f', 3)
                       .arg(gyro_x, 9, 'f', 3)
                       .arg(gyro_y, 9, 'f', 3)
                       .arg(gyro_z, 9, 'f', 3);
    ui->label_imu_status->setText(text);
}

void MainWindow::_update_odom_label(double line_v, double angle_v, double pos_x, double pos_y, double yaw_deg) {
    QString text = QString(
                       "速度:\n"
                       "    v = %1 m/s\n"
                       "    w = %2 rad/s\n"
                       "位置:\n"
                       "    x = %3 m\n"
                       "    y = %4 m\n"
                       "角度:\n"
                       "    z = %5 °")
                       .arg(line_v, 9, 'f', 3)
                       .arg(angle_v, 9, 'f', 3)
                       .arg(pos_x, 9, 'f', 3)
                       .arg(pos_y, 9, 'f', 3)
                       .arg(yaw_deg, 9, 'f', 2);
    ui->label_odom_msg->setText(text);
}

void MainWindow::_update_cmd_status_label(const QString &cmd_string, int state) {
    QString html_string, cmd_string_tmp;
    if (state == CommandState::Success) {
        cmd_string_tmp = cmd_string + "执行成功！";
        cmd_status_label_->setStyleSheet("color:white;background-color:green;font-size:20px;");
        html_string = QString("<span style='color:green;'>%1</span>").arg(cmd_string_tmp.toHtmlEscaped());
    } else if (state == CommandState::Add) {
        cmd_string_tmp = cmd_string + "等待执行...";
        cmd_status_label_->setStyleSheet("color:white;background-color:#FFB300;font-size:20px;");
        html_string = QString("<span style='color:#FFB300;'>%1</span>").arg(cmd_string_tmp.toHtmlEscaped()); //
    } else if (state == CommandState::Running) {
        cmd_string_tmp = cmd_string + "运行中...";
        cmd_status_label_->setStyleSheet("color:white;background-color:#CCCC00;font-size:20px;");
        html_string = QString("<span style='color:#CCCC00;'>%1</span>").arg(cmd_string_tmp.toHtmlEscaped()); //
    } else if (state == CommandState::Fail) {
        cmd_string_tmp = cmd_string + "执行失败.";
        cmd_status_label_->setStyleSheet("color:white;background-color:red;font-size:20px;");
        html_string = QString("<span style='color:red;'>%1</span>").arg(cmd_string_tmp.toHtmlEscaped());
    } else if (state == CommandState::SerialMsg) {
        cmd_string_tmp = _get_cmd_string_prefix() + cmd_string;
        html_string = QString("<span style='color:black;'>%1</span>").arg(cmd_string_tmp.toHtmlEscaped());
    }

    if (state == CommandState::Success || state == CommandState::Fail || state == CommandState::Add || state == CommandState::Running || state == CommandState::SerialMsg) {
        ui->textEdit_cmd->append(html_string);
        ui->textEdit_cmd->moveCursor(QTextCursor::End);
    }

    if (state == CommandState::Success || state == CommandState::Fail || state == CommandState::Running) {
        cmd_status_label_->setText(cmd_string_tmp);
        cmd_status_timer_->start();
    }
}

void MainWindow::_update_speed_percent_label(double percent) {
    ui->label_speed_percent->setText(QString::number(percent * 100, 'f', 1) + "%");
}

// void MainWindow::_readConfigJson() {
//     QFile file(CONFIG_FILE_PATH);
//     if (file.exists()) {
//         if (!file.open(QIODevice::ReadOnly)) {
//             QMessageBox::critical(this, "错误", "读取配置文件失败");
//             return;
//         }

//         QByteArray data = file.readAll();
//         file.close();

//         QJsonParseError error;
//         QJsonDocument doc = QJsonDocument::fromJson(data, &error);
//         if (error.error != QJsonParseError::NoError) {
//             QMessageBox::critical(this, "错误", "读取配置文件失败");
//             return;
//         }

//         if (!doc.isObject()) {
//             QMessageBox::critical(this, "错误", "读取配置文件失败");
//             return;
//         }

//         QJsonObject obj = doc.object();
//         ui->doubleSpinBox_p->setValue(obj["p"].toDouble());
//         ui->doubleSpinBox_i->setValue(obj["i"].toDouble());
//         ui->doubleSpinBox_d->setValue(obj["d"].toDouble());
//         ui->doubleSpinBox_max_i->setValue(obj["max_total_i"].toDouble());
//         ui->spinBox_millseconds->setValue(obj["millseconds"].toInt());
//         ui->doubleSpinBox_max_v->setValue(obj["max_v"].toDouble());
//         ui->doubleSpinBox_max_acc->setValue(obj["max_acc"].toDouble());
//         ui->doubleSpinBox_jerk->setValue(obj["jerk"].toDouble());
//     }
// }

// void MainWindow::_writeConfigJson() {
//     QJsonObject obj;
//     obj["p"] = ui->doubleSpinBox_p->value();
//     obj["i"] = ui->doubleSpinBox_i->value();
//     obj["d"] = ui->doubleSpinBox_d->value();
//     obj["max_total_i"] = ui->doubleSpinBox_max_i->value();
//     obj["millseconds"] = ui->spinBox_millseconds->value();
//     obj["max_v"] = ui->doubleSpinBox_max_v->value();
//     obj["max_acc"] = ui->doubleSpinBox_max_acc->value();
//     obj["jerk"] = ui->doubleSpinBox_jerk->value();

//     QJsonDocument doc(obj);

//     QString configFolder = QFileInfo(CONFIG_FILE_PATH).absolutePath();
//     QDir dir(configFolder);
//     if (!dir.exists()) {
//         dir.mkpath("."); // 创建目录
//     }

//     QFile file(CONFIG_FILE_PATH);
//     if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
//         QMessageBox::critical(this, "错误", "写入配置文件失败");
//         return;
//     }
//     file.write(doc.toJson(QJsonDocument::Indented)); // 或 Compact
//     file.close();
// }
