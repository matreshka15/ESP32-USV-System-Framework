#include "motor.h"
#include "esp_log.h"

static const char *TAG = "Motor";
static motor_t motors[MOTOR_NUM];

void motor_init(int motor1_pin, int motor2_pin)
{
    // 配置PWM定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 初始化电机1
    motors[0].gpio_num = motor1_pin;
    motors[0].channel = LEDC_CHANNEL_0;
    
    ledc_channel_config_t channel_conf = {
        .gpio_num = motor1_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // 初始化电机2
    motors[1].gpio_num = motor2_pin;
    motors[1].channel = LEDC_CHANNEL_1;
    
    channel_conf.gpio_num = motor2_pin;
    channel_conf.channel = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    ESP_LOGI(TAG, "电机控制模块初始化完成");
}

void set_motor_speed(int motor_id, int speed)
{
    if (motor_id < 0 || motor_id >= MOTOR_NUM) return;
    
    // 限制速度范围0-100%
    speed = speed < 0 ? 0 : (speed > 100 ? 100 : speed);
    
    // 计算PWM占空比 (电调通常需要1000-2000us脉冲宽度)
    uint32_t duty = (1024 * (1000 + speed * 10)) / 20000;
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].channel));
    
    motors[motor_id].speed = speed;
}