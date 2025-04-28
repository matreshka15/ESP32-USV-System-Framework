#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "kalman_filter.h"

// 模块头文件
#include "gps.h"
#include "imu.h"
#include "motor.h"
#include "navigation.h"
#include "wifi.h"
#include "remote.h"

static const char *TAG = "Main";

// 定义系统模式
typedef enum {
    SYSTEM_STANDBY,
    SYSTEM_REMOTE,
    SYSTEM_AUTO_NAV
} system_mode_t;

// 全局变量
static QueueHandle_t remote_queue = NULL;
static system_mode_t current_mode = SYSTEM_STANDBY;
static KalmanFilter course_filter;

// 在main.c中的初始化部分
void app_main()
{
    // 遥控器输入引脚定义 (根据实际接线修改)
    int remote_pins[REMOTE_CHANNEL_NUM] = {12, 13, 14, 15}; // GPIO12-15
    
    // 初始化卡尔曼滤波器
    kalman_filter_init(&course_filter, 0.01, 0.1, 0, 1);
    
    // 初始化遥控模块
    remote_init(remote_pins, REMOTE_WFR07);
    
    // 设置电机控制回调
    set_motor_speed_cb(set_motor_speed);
    
    // 创建遥控任务
    xTaskCreate(remote_task, "remote_task", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "无人船控制系统启动");
    
    // 创建队列
    gps_queue = xQueueCreate(5, sizeof(gps_data_t));
    imu_queue = xQueueCreate(5, sizeof(imu_data_t));
    remote_queue = xQueueCreate(5, sizeof(remote_cmd_t));
    
    // 初始化各模块
    gps_init(gps_queue);
    imu_init(imu_queue);
    motor_init();
    wifi_init(WIFI_MODE_AP);
    
    // 设置WiFi回调
    set_start_navigation_cb(start_navigation);
    set_stop_navigation_cb(stop_navigation);
    set_add_waypoint_cb(add_waypoint);
    set_remove_waypoint_cb(remove_waypoint);

}

// 无人船控制任务
void usv_control_task(void *pvParameters)
{
    remote_cmd_t remote_cmd;
    
    while(1) {
        // 获取遥控指令
        if(xQueueReceive(remote_queue, &remote_cmd, 0) == pdTRUE) {
            if(remote_cmd.mode_switch) {
                current_mode = (current_mode + 1) % 3;
                ESP_LOGI(TAG, "切换模式到: %d", current_mode);
            }
        }
        
        // 根据当前模式执行相应操作
        switch(current_mode) {
            case SYSTEM_STANDBY:
                // 系统待机模式，停止所有电机
                set_motor_speed(0, 0);
                set_motor_speed(1, 0);
                ESP_LOGI(TAG, "系统处于待机模式，电机已停止");
                break;
                
            case SYSTEM_REMOTE:
                // 获取最新的遥控指令
                if(xQueueReceive(remote_queue, &remote_cmd, 0) == pdTRUE) {
                    // 假设 remote_cmd 包含 left_motor_speed 和 right_motor_speed 字段
                    set_motor_speed(0, remote_cmd.left_motor_speed);
                    set_motor_speed(1, remote_cmd.right_motor_speed);
                    ESP_LOGI(TAG, "远程控制模式，左电机速度: %d，右电机速度: %d", 
                             remote_cmd.left_motor_speed, remote_cmd.right_motor_speed);
                }
                break;
                
            case SYSTEM_AUTO_NAV:
                if (is_navigating && current_waypoint) {
                    gps_data = get_current_gps_data();
                    imu_data = get_current_imu_data(); // 假设存在该函数

                    // 融合imu和GPS数据
                    float fused_course = kalman_filter_update(&course_filter, gps_data.course);
                    fused_course = kalman_filter_update(&course_filter, imu_data.yaw);

                    waypoint_t current = {gps_data.latitude, gps_data.longitude};
                    waypoint_t target = current_waypoint->data;

                    float distance = calculate_distance(current, target);
                    float bearing = calculate_bearing(current, target);

                    ESP_LOGI(TAG, "目标点: %.6f, %.6f 距离: %.1f米 方位角: %.1f°",
                             target.latitude, target.longitude, distance, bearing);

                    // 简单导航控制逻辑
                    if (distance < 5.0) { // 到达目标点
                        current_waypoint = current_waypoint->next;
                        if (!current_waypoint) {
                            stop_navigation();
                            ESP_LOGI(TAG, "已完成所有路径点导航");
                        }
                    } else {
                        // 简单的转向控制
                        float error = bearing - fused_course;
                        if (error > 180) error -= 360;
                        if (error < -180) error += 360;

                        int base_speed = 50;
                        int left_speed = base_speed;
                        int right_speed = base_speed;

                        if (error > 10) {
                            left_speed -= 20;
                        } else if (error < -10) {
                            right_speed -= 20;
                        }

                        set_motor_speed(0, left_speed);
                        set_motor_speed(1, right_speed);
                    }
                }
                break;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}