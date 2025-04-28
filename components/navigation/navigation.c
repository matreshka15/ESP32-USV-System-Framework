#include "navigation.h"
#include "gps.h"
#include "motor.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "Navigation";
static waypoint_node_t *waypoint_list = NULL;
static int waypoint_count = 0;
static waypoint_node_t *current_waypoint = NULL;
static bool is_navigating = false;

void navigation_init()
{
    clear_waypoints();
    is_navigating = false;
    ESP_LOGI(TAG, "导航模块初始化完成");
}

bool add_waypoint(double lat, double lon)
{
    waypoint_node_t *new_node = (waypoint_node_t *)malloc(sizeof(waypoint_node_t));
    if (!new_node) {
        ESP_LOGE(TAG, "内存分配失败");
        return false;
    }

    new_node->data.latitude = lat;
    new_node->data.longitude = lon;
    new_node->next = NULL;

    if (!waypoint_list) {
        waypoint_list = new_node;
        current_waypoint = new_node;
    } else {
        waypoint_node_t *temp = waypoint_list;
        while (temp->next) {
            temp = temp->next;
        }
        temp->next = new_node;
    }

    waypoint_count++;
    ESP_LOGI(TAG, "添加路径点: %.6f, %.6f", lat, lon);
    return true;
}

bool remove_waypoint(int index)
{
    if (index < 0 || index >= waypoint_count) {
        return false;
    }

    waypoint_node_t *current = waypoint_list;
    waypoint_node_t *prev = NULL;

    if (index == 0) {
        waypoint_list = current->next;
        if (current_waypoint == current) {
            current_waypoint = waypoint_list;
        }
        free(current);
    } else {
        for (int i = 0; i < index; i++) {
            prev = current;
            current = current->next;
        }
        prev->next = current->next;
        if (current_waypoint == current) {
            current_waypoint = prev->next;
        }
        free(current);
    }

    waypoint_count--;
    return true;
}

void clear_waypoints()
{
    waypoint_node_t *current = waypoint_list;
    waypoint_node_t *next;

    while (current) {
        next = current->next;
        free(current);
        current = next;
    }

    waypoint_list = NULL;
    current_waypoint = NULL;
    waypoint_count = 0;
}

void start_navigation()
{
    if (waypoint_count == 0) {
        ESP_LOGE(TAG, "没有可用的路径点");
        return;
    }

    is_navigating = true;
    current_waypoint = waypoint_list;
    ESP_LOGI(TAG, "开始自动导航，共有%d个路径点", waypoint_count);
}

void stop_navigation()
{
    is_navigating = false;
    set_motor_speed(0, 0);
    set_motor_speed(1, 0);
    ESP_LOGI(TAG, "停止自动导航");
}

waypoint_t get_current_waypoint()
{
    if (current_waypoint) {
        return current_waypoint->data;
    }

    waypoint_t empty = {0, 0};
    return empty;
}

float calculate_distance(waypoint_t p1, waypoint_t p2)
{
    // 使用Haversine公式计算两点间距离
    double lat1 = p1.latitude * M_PI / 180.0;
    double lon1 = p1.longitude * M_PI / 180.0;
    double lat2 = p2.latitude * M_PI / 180.0;
    double lon2 = p2.longitude * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2) * sin(dlon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return 6371000 * c; // 地球半径6371km
}

float calculate_bearing(waypoint_t p1, waypoint_t p2)
{
    double lat1 = p1.latitude * M_PI / 180.0;
    double lon1 = p1.longitude * M_PI / 180.0;
    double lat2 = p2.latitude * M_PI / 180.0;
    double lon2 = p2.longitude * M_PI / 180.0;

    double dlon = lon2 - lon1;

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);

    float bearing = atan2(y, x) * 180.0 / M_PI;

    // 转换为0-360度
    bearing = fmod(bearing + 360, 360);

    return bearing;
}

// 读取所有导航点
waypoint_node_t* read_all_waypoints()
{
    return waypoint_list;
}