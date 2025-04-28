#ifndef WIFI_H
#define WIFI_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"


// WiFi初始化
typedef enum {
    WIFI_MODE_AP,
    WIFI_MODE_STA
} wifi_mode_t;

// 更新初始化函数声明
void wifi_init(wifi_mode_t mode);

// WiFi任务
void wifi_task(void *pvParameters);

// 启动HTTP服务器
void start_http_server();

// 停止HTTP服务器
void stop_http_server();

// 添加路径点回调函数
typedef void (*add_waypoint_cb_t)(double lat, double lon);

// 设置添加路径点回调
void set_add_waypoint_cb(add_waypoint_cb_t cb);

// 删除路径点回调函数
typedef void (*remove_waypoint_cb_t)(int index);

// 设置删除路径点回调
void set_remove_waypoint_cb(remove_waypoint_cb_t cb);

// 开始导航回调函数
typedef void (*start_navigation_cb_t)();

// 设置开始导航回调
void set_start_navigation_cb(start_navigation_cb_t cb);

// 停止导航回调函数
typedef void (*stop_navigation_cb_t)();

// 设置停止导航回调
void set_stop_navigation_cb(stop_navigation_cb_t cb);

#endif