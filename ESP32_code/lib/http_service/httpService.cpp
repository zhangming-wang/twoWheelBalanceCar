#include "httpService.h"

HttpService::HttpService(uint port) : webServer_(port) {
}

HttpService::~HttpService() {
}

void HttpService::begin() {
    webServer_.on("/ping", [this]() {
        webServer_.sendHeader("Access-Control-Allow-Origin", "*");
        webServer_.send(200, "text/plain", "connected");
    });
    webServer_.on("/restart", [this]() {
        twoWheelDiffModel.restart();
    });
    webServer_.on("/brake", [this]() {
        twoWheelDiffModel.brake();
    });
    webServer_.on("/stop_move", [this]() {
        twoWheelDiffModel.stop_move();
    });
    webServer_.on("/move_front", [this]() {
        twoWheelDiffModel.move_front();
    });
    webServer_.on("/move_back", [this]() {
        twoWheelDiffModel.move_back();
    });
    webServer_.on("/move_left", [this]() {
        twoWheelDiffModel.move_left();
    });
    webServer_.on("/move_right", [this]() {
        twoWheelDiffModel.move_right();
    });
    webServer_.on("/save", [this]() {
        twoWheelDiffModel.save_params();
        twoWheelDiffModel.save_settings();
    });
    webServer_.on("/set_speed_percent", [this]() {
        auto percent = webServer_.arg("value").toFloat();
        twoWheelDiffModel.set_speed_percent(percent);
    });
    webServer_.on("/set_motion_mode", [this]() {
        auto mode = webServer_.arg("value").toInt();
        twoWheelDiffModel.set_motion_mode(mode);
    });
    webServer_.on("/get_data", [this]() {
        webServer_.send(200, "application/json", twoWheelDiffModel.get_http_data());
    });
    webServer_.onNotFound([this]() {
        if (webServer_.method() == HTTP_OPTIONS) {
            webServer_.sendHeader("Access-Control-Allow-Origin", "*");
            webServer_.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            webServer_.sendHeader("Access-Control-Allow-Headers", "*");
            webServer_.send(204); // No Content
        } else {
            webServer_.sendHeader("Access-Control-Allow-Origin", "*");
            webServer_.send(404, "text/plain", "404 Not Found");
        }
    });

    webServer_.begin();
}

void HttpService::stop() {
    webServer_.stop();
}

void HttpService::start_task() {
    if (enable_task_run == false) {
        enable_task_run = true;
        xTaskCreate(http_serive_task, "http_serive_task", 8192, this, 1, NULL);
    }
}

void HttpService::stop_task() {
    if (enable_task_run) {
        enable_task_run = false;
    }
}

void HttpService::handleClient() {
    webServer_.handleClient();
}

void http_serive_task(void *args) {
    HttpService *httpService = static_cast<HttpService *>(args);
    bool connected = false;
    while (httpService->enable_task_run) {
        if (WiFi.status() == WL_CONNECTED) {
            if (connected == false) {
                connected = true;
                httpService->begin();
                serial_print("HTTP Server started!");
            }
            httpService->handleClient(); // 处理HTTP请求
        } else {
            if (connected) {
                connected = false;
                httpService->stop();
                serial_print("HTTP Server stopped due to Wi-Fi disconnect");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(twoWheelDiffModel.get_milliseconds()));
    }
    if (connected) {
        httpService->stop();
        serial_print("HTTP Server stopped on task exit");
    }
    vTaskDelete(NULL);
}
