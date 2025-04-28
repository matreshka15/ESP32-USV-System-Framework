#include "gps.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "GPS";
static gps_data_t current_gps_data;

void gps_init(int tx_pin, int rx_pin)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUFFER_SIZE, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "GPS模块初始化完成");
}

bool parse_nmea(const char *nmea, gps_data_t *data)
{
    // 示例解析GPRMC和GPGGA语句
    if (strstr(nmea, "$GPRMC")) {
        char status;
        float lat, lon;
        char lat_dir, lon_dir;
        float speed_knots, course;
        
        if (sscanf(nmea, "$GPRMC,%*f,%c,%f,%c,%f,%c,%f,%f,", 
                  &status, &lat, &lat_dir, &lon, &lon_dir, &speed_knots, &course) >= 6) {
            
            data->valid = (status == 'A');
            // 更精确的经纬度转换
            data->latitude = (int)(lat / 100) + (lat - (int)(lat / 100) * 100) / 60.0;
            data->longitude = (int)(lon / 100) + (lon - (int)(lon / 100) * 100) / 60.0;
            data->speed = speed_knots;
            data->course = course;
            
            if (lat_dir == 'S') data->latitude = -data->latitude;
            if (lon_dir == 'W') data->longitude = -data->longitude;
            
            return true;
        }
    } else if (strstr(nmea, "$GPGGA")) {
        // 可以添加GPGGA语句的解析逻辑
    }
    return false;
}

void gps_task(void *pvParameters)
{
    uint8_t data[GPS_BUFFER_SIZE];
    
    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, GPS_BUFFER_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            if (parse_nmea((char *)data, &current_gps_data)) {
                ESP_LOGI(TAG, "位置: %.6f, %.6f 速度: %.1f节 航向: %.1f°", 
                         current_gps_data.latitude, current_gps_data.longitude, 
                         current_gps_data.speed, current_gps_data.course);
            } else {
                ESP_LOGW(TAG, "无法解析NMEA数据: %s", (char *)data);
            }
        } else if (len < 0) {
            ESP_LOGE(TAG, "读取串口数据出错");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 调整延迟时间
    }
}

// 获取当前GPS数据
gps_data_t get_current_gps_data(void)
{
    return current_gps_data;
}