#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include "driver/uart.h"

#define GPS_UART_NUM UART_NUM_1
#define GPS_BUFFER_SIZE 1024

typedef struct {
    double latitude;  // 纬度
    double longitude; // 经度
    float speed;      // 速度(knots)
    float course;      // 航向
    bool valid;        // 数据是否有效
} gps_data_t;

// 初始化GPS模块
void gps_init(int tx_pin, int rx_pin);
// 获取当前GPS数据
gps_data_t get_current_gps_data(void);

#endif