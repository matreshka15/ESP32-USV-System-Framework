#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "driver/gpio.h"

#define MOTOR_NUM 2
#define MOTOR_PWM_FREQ 400
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT

typedef struct {
    int gpio_num;          // 电机控制GPIO
    ledc_channel_t channel; // PWM通道
    int speed;             // 当前速度(0-100%)
} motor_t;

// 初始化电机控制模块
void motor_init(int motor1_pin, int motor2_pin);

// 设置电机速度
void set_motor_speed(int motor_id, int speed);

// 电机控制任务
void motor_task(void *pvParameters);

#endif