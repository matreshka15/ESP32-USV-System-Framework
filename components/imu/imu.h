#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include "driver/i2c.h"

typedef struct {
    float accel_x, accel_y, accel_z;  // 加速度(m/s²)
    float gyro_x, gyro_y, gyro_z;     // 角速度(rad/s)
    float mag_x, mag_y, mag_z;        // 磁力计数据(μT)
    float roll, pitch, yaw;           // 欧拉角(度)
    bool valid;                       // 数据是否有效
} imu_data_t;

// 初始化IMU模块
void imu_init(int sda_pin, int scl_pin);

// IMU任务
void imu_task(void *pvParameters);

// 卡尔曼滤波姿态解算
void kalman_filter_update(imu_data_t *data);

// 获取当前IMU数据
imu_data_t get_current_imu_data(void);

#endif