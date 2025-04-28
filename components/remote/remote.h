#ifndef REMOTE_H
#define REMOTE_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "motor.h"

#define REMOTE_CHANNEL_NUM 4  // WFR07有4个通道
#define REMOTE_PWM_MIN 1000   // 最小脉宽(us)
#define REMOTE_PWM_MAX 2000   // 最大脉宽(us)

typedef enum {
    REMOTE_WFR07,
    REMOTE_WFT07
} remote_type_t;

// 遥控通道定义
typedef enum {
    CH_THROTTLE = 0,  // 油门通道
    CH_STEERING = 1,  // 转向通道
    CH_AUX1 = 2,      // 辅助通道1
    CH_AUX2 = 3       // 辅助通道2
} remote_channel_t;

// 初始化遥控模块
void remote_init(int *input_pins, remote_type_t type);

// 获取指定通道的PWM脉宽(us)
uint16_t remote_get_pulse_width(remote_channel_t channel);

// 获取指定通道的标准化值(0-100%)
uint8_t remote_get_channel_value(remote_channel_t channel);

typedef struct {
    int left_speed;
    int right_speed;
    bool mode_switch;
} remote_cmd_t;

// 初始化遥控模块
void remote_init(int tx_pin, int rx_pin, remote_type_t type);

// 遥控任务
void remote_task(void *pvParameters);

// 设置电机控制回调
typedef void (*set_motor_speed_cb_t)(int motor_id, int speed);

// 设置电机控制回调函数
void set_motor_speed_cb(set_motor_speed_cb_t cb);



#endif