#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>
#include "gps.h"

#define MAX_WAYPOINTS 50

typedef struct {
    double latitude;
    double longitude;
} waypoint_t;

// 定义链表节点结构体
typedef struct waypoint_node {
    waypoint_t data;
    struct waypoint_node *next;
} waypoint_node_t;

// 初始化导航模块
void navigation_init();

// 导航任务
void navigation_task(void *pvParameters);

// 添加路径点
bool add_waypoint(double lat, double lon);

// 删除路径点
bool remove_waypoint(int index);

// 清除所有路径点
void clear_waypoints();

// 开始自动导航
void start_navigation();

// 停止自动导航
void stop_navigation();

// 获取当前路径点
waypoint_t get_current_waypoint();

// 计算两点间距离(米)
float calculate_distance(waypoint_t p1, waypoint_t p2);

// 计算两点间方位角(度)
float calculate_bearing(waypoint_t p1, waypoint_t p2);

// 读取所有导航点
waypoint_node_t* read_all_waypoints();

#endif