#include "remote.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include <string.h>

static const char *TAG = "Remote";
static remote_type_t remote_type;
static set_motor_speed_cb_t motor_speed_cb = NULL;
static int input_pins[REMOTE_CHANNEL_NUM];
static pcnt_unit_t pcnt_units[REMOTE_CHANNEL_NUM];

void remote_init(int *pins, remote_type_t type)
{
    remote_type = type;
    memcpy(input_pins, pins, sizeof(input_pins));
    
    // 初始化PCNT(脉冲计数器)用于测量PWM脉宽
    for(int i = 0; i < REMOTE_CHANNEL_NUM; i++) {
        pcnt_config_t pcnt_config = {
            .pulse_gpio_num = pins[i],
            .ctrl_gpio_num = -1,
            .lctrl_mode = PCNT_MODE_KEEP,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_INC,
            .neg_mode = PCNT_COUNT_DIS,
            .counter_h_lim = 20000, // 最大测量20ms周期
            .counter_l_lim = 0
        };
        
        pcnt_unit_config(&pcnt_config);
        pcnt_set_filter_value(i, 100); // 设置滤波器(100个时钟周期)
        pcnt_filter_enable(i);
        pcnt_counter_pause(i);
        pcnt_counter_clear(i);
        pcnt_counter_resume(i);
        
        pcnt_units[i] = i;
    }
    
    ESP_LOGI(TAG, "WFR07遥控模块初始化完成，使用PWM脉宽测量");
}

uint16_t remote_get_pulse_width(remote_channel_t channel)
{
    if(channel >= REMOTE_CHANNEL_NUM) return 0;
    
    int16_t pulse_width = 0;
    pcnt_get_counter_value(pcnt_units[channel], &pulse_width);
    return (uint16_t)pulse_width;
}

uint8_t remote_get_channel_value(remote_channel_t channel)
{
    uint16_t pulse_width = remote_get_pulse_width(channel);
    pulse_width = pulse_width < REMOTE_PWM_MIN ? REMOTE_PWM_MIN : 
                 (pulse_width > REMOTE_PWM_MAX ? REMOTE_PWM_MAX : pulse_width);
    
    // 将脉宽映射到0-100%范围
    return (uint8_t)((pulse_width - REMOTE_PWM_MIN) * 100 / (REMOTE_PWM_MAX - REMOTE_PWM_MIN));
}

// 保留原有的回调设置函数
void set_motor_speed_cb(set_motor_speed_cb_t cb)
{
    motor_speed_cb = cb;
}

void remote_task(void *pvParameters)
{
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    remote_cmd_t cmd = {0};
    
    while(1) {
        // 读取油门和转向通道
        uint8_t throttle = remote_get_channel_value(CH_THROTTLE);
        uint8_t steering = remote_get_channel_value(CH_STEERING);
        uint8_t aux1 = remote_get_channel_value(CH_AUX1);
        
        // 转换为电机速度
        cmd.left_speed = throttle;
        cmd.right_speed = throttle;
        
        if(steering > 50) { // 右转
            cmd.right_speed -= (steering - 50) * 2;
        } else if(steering < 50) { // 左转
            cmd.left_speed -= (50 - steering) * 2;
        }
        
        // 使用辅助通道1作为模式切换按钮
        cmd.mode_switch = (aux1 > 50);
        
        // 发送到队列
        xQueueSend(queue, &cmd, 0);
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}