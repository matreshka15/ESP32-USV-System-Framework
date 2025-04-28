#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "kalman_filter.h" // 包含卡尔曼滤波头文件

#define MPU6050_ADDR 0x68
#define HMC5883L_ADDR 0x1E

static const char *TAG = "IMU";
static imu_data_t current_imu_data;

// 提取 I2C 通信公共部分
static esp_err_t i2c_read_registers(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void imu_init(int sda_pin, int scl_pin)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    
    // 初始化MPU6050
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // PWR_MGMT_1
    i2c_master_write_byte(cmd, 0x00, true); // 唤醒
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    
    // 初始化HMC5883L
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, HMC5883L_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x02, true); // Mode register
    i2c_master_write_byte(cmd, 0x00, true); // Continuous measurement mode
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    
    ESP_LOGI(TAG, "IMU模块初始化完成");
}

void read_mpu6050_data()
{
    uint8_t data[14];
    esp_err_t ret = i2c_read_registers(MPU6050_ADDR, 0x3B, data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
        return;
    }
    
    current_imu_data.accel_x = (int16_t)((data[0] << 8) | data[1]) / 16384.0 * 9.8;
    current_imu_data.accel_y = (int16_t)((data[2] << 8) | data[3]) / 16384.0 * 9.8;
    current_imu_data.accel_z = (int16_t)((data[4] << 8) | data[5]) / 16384.0 * 9.8;
    
    current_imu_data.gyro_x = (int16_t)((data[8] << 8) | data[9]) / 131.0 * (M_PI / 180.0);
    current_imu_data.gyro_y = (int16_t)((data[10] << 8) | data[11]) / 131.0 * (M_PI / 180.0);
    current_imu_data.gyro_z = (int16_t)((data[12] << 8) | data[13]) / 131.0 * (M_PI / 180.0);
}

void read_hmc5883l_data()
{
    uint8_t data[6];
    esp_err_t ret = i2c_read_registers(HMC5883L_ADDR, 0x03, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read HMC5883L data");
        return;
    }
    
    current_imu_data.mag_x = (int16_t)((data[0] << 8) | data[1]) * 0.92;
    current_imu_data.mag_y = (int16_t)((data[2] << 8) | data[3]) * 0.92;
    current_imu_data.mag_z = (int16_t)((data[4] << 8) | data[5]) * 0.92;
}

void imu_kalman_filter_update(imu_data_t *data)
{
    // 假设已经有卡尔曼滤波器实例
    static KalmanFilter kf_roll, kf_pitch, kf_yaw;
    static bool initialized = false;
    if (!initialized) {
        kalman_filter_init(&kf_roll, 0.001f, 0.1f, 0.0f, 1.0f);
        kalman_filter_init(&kf_pitch, 0.001f, 0.1f, 0.0f, 1.0f);
        kalman_filter_init(&kf_yaw, 0.001f, 0.1f, 0.0f, 1.0f);
        initialized = true;
    }

    // 加速度计计算姿态
    float accel_roll = atan2(data->accel_y, data->accel_z) * 180.0 / M_PI;
    float accel_pitch = atan2(-data->accel_x, sqrt(data->accel_y * data->accel_y + data->accel_z * data->accel_z)) * 180.0 / M_PI;

    // 使用卡尔曼滤波更新姿态
    data->roll = kalman_filter_update(&kf_roll, accel_roll);
    data->pitch = kalman_filter_update(&kf_pitch, accel_pitch);
    // 磁力计计算偏航角
    float mag_x = data->mag_x * cos(data->pitch * M_PI / 180.0) + 
                 data->mag_z * sin(data->pitch * M_PI / 180.0);
    float mag_y = data->mag_x * sin(data->roll * M_PI / 180.0) * sin(data->pitch * M_PI / 180.0) + 
                 data->mag_y * cos(data->roll * M_PI / 180.0) - 
                 data->mag_z * sin(data->roll * M_PI / 180.0) * cos(data->pitch * M_PI / 180.0);
    float raw_yaw = atan2(-mag_y, mag_x) * 180.0 / M_PI;
    data->yaw = kalman_filter_update(&kf_yaw, raw_yaw);

    data->valid = true;
}

void imu_task(void *pvParameters)
{
    while (1) {
        read_mpu6050_data();
        read_hmc5883l_data();
        imu_kalman_filter_update(&current_imu_data);
        
        ESP_LOGI(TAG, "姿态: 横滚=%.1f° 俯仰=%.1f° 偏航=%.1f°", 
                 current_imu_data.roll, current_imu_data.pitch, current_imu_data.yaw);
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// 获取当前IMU数据
imu_data_t get_current_imu_data(void) {
    return current_imu_data;
}