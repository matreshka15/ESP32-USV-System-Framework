# 无人船控制系统

* 作者的无人船专栏：[知乎专栏](https://www.zhihu.com/column/c_1898816235065578314)
* 发电说明：若您觉得本项目有帮助，欢迎支持作者：[爱发电](https://afdian.com/a/hankli)

## 项目概述

本项目是一个无人船控制系统框架，旨在帮助开发者实现无人船的自主导航和控制功能。框架的示例系统以 ESP32S3 开发板为主控板，集成了多种传感器（如 MPU6050 加速度计/陀螺仪、HMC5883L 磁力计、Ublox M8 GPS 模块）来获取无人船的姿态和位置信息。通过卡尔曼滤波对传感器数据进行处理，提高数据的准确性和稳定性。同时，系统支持 WiFi 通信和遥控器控制，可实现远程操作和导航。执行机构为 2 个直流电机，通过 PWM 信号进行控制。

## 项目亮点
- **多传感器融合技术**：集成了 MPU6050 加速度计/陀螺仪、HMC5883L 磁力计和 Ublox M8 GPS 模块，通过卡尔曼滤波和 Madgwick 算法融合多传感器数据，显著提高了姿态和位置信息的准确性与稳定性。
- **灵活的控制方式**：支持 WiFi 通信和遥控器控制两种方式，可根据实际需求灵活选择，实现无人船的远程操作和导航。
- **可扩展性强**：提供了详细的硬件和软件定制化指南，方便用户更换传感器、增加电机、修改控制算法等，能够轻松适应不同的应用场景。
- **高效的开发环境**：基于 ESP-IDF v4.4+ 开发环境，结合 CMake 3.16+ 和 Python 3.8+，开发流程高效，便于代码的管理和维护。

## 硬件需求
- **主控板**：ESP32S3开发板
- **传感器**：
  - MPU6050加速度计/陀螺仪
  - HMC5883L磁力计
  - Ublox M8 GPS模块
- **执行机构**：2个直流电机(带驱动板)
- **通信模块**：WiFi模块(ESP32内置)
- **遥控器**：支持WFR07或WFT07型号

## 软件需求
- **开发环境**：
  - ESP-IDF v4.4+
  - CMake 3.16+
  - Python 3.8+
- **工具链**：
  - ESP-IDF工具链
  - Git
  - Serial terminal (如Putty)

## 硬件连接
| 模块        | 引脚连接           | 备注                  |
|-------------|--------------------|-----------------------|
| MPU6050     | SDA: GPIO21        | I2C0接口              |
|             | SCL: GPIO22        |                       |
| HMC5883L    | 共用I2C0接口       | 地址0x1E              |
| GPS模块     | TX: GPIO17         | UART1, 波特率9600     |
|             | RX: GPIO16         |                       |
| 电机1       | PWM: GPIO25        |                       |
| 电机2       | PWM: GPIO26        |                       |
| 遥控器通道1 | GPIO12             | 模式切换通道          |
| 遥控器通道2 | GPIO13             | 左电机控制            |
| 遥控器通道3 | GPIO14             | 右电机控制            |
| 遥控器通道4 | GPIO15             | 备用通道              |

## 环境搭建步骤
1. 安装ESP-IDF开发环境：
```bash
git clone -b v4.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```
2. 克隆本仓库：
```bash
git clone https://github.com/matreshka15/ESP32-USV-System-Framework.git
cd USV
```
3. 配置项目：
```bash
idf.py menuconfig
```
4. 编译并烧录：
```bash
idf.py build
idf.py -p COMx flash monitor
```

## 定制化指南

### 1. 硬件定制
#### 更换传感器
当需要更换传感器时，主要修改 `components/imu/imu.c` 中的初始化代码。例如，如果要将 MPU6050 更换为其他型号的加速度计/陀螺仪，需要更新传感器的初始化参数、I2C 地址和寄存器读取方式。具体步骤如下：
1. 找到 `read_mpu6050_data` 函数，修改传感器地址、寄存器起始地址和数据读取逻辑。
2. 调整数据转换公式，以适应新传感器的输出格式。

#### 增加电机
若要增加电机，需要扩展 `components/motor/motor.c` 中的 PWM 通道配置。步骤如下：
1. 定义新的 GPIO 引脚用于新增电机的 PWM 控制。
2. 在电机初始化函数中，添加新的 PWM 通道配置代码。
3. 扩展电机控制函数，以支持对新增电机的控制。

#### 更换遥控器
更换遥控器时，需要修改 `components/remote/remote.c` 中的解码逻辑。具体操作如下：
1. 根据新遥控器的信号格式，修改信号解码函数。
2. 调整引脚配置，确保新遥控器的信号能被正确接收。

### 2. 软件定制
#### 修改控制算法
若要修改控制算法，需要编辑 `components/navigation/navigation.c` 文件。该文件包含了无人船的导航控制逻辑，你可以根据需求：
1. 调整路径规划算法，如修改 GPS 坐标点的处理逻辑。
2. 优化姿态控制算法，提高无人船的航行稳定性。

#### 调整滤波参数
调整滤波参数可在 `components/kalman/kalman_filter.c` 中修改 Q/R 值。Q 值表示过程噪声协方差，R 值表示测量噪声协方差。增大 Q 值会使滤波器更相信预测值，增大 R 值会使滤波器更相信测量值。你可以根据实际的传感器噪声情况进行调整。

#### 自定义 WiFi 配置
自定义 WiFi 配置需要修改 `components/wifi/wifi_config.h` 文件。在该文件中，你可以：
1. 修改 WiFi 的 SSID 和密码。
2. 调整 WiFi 连接的超时时间和重试次数。
3. 配置 WiFi 模式，如 AP 模式或 Station 模式。

### 3. 功能扩展
#### 添加新传感器
添加新传感器需要以下步骤：
1. 在 `components` 目录下创建新模块，例如 `components/new_sensor`。
2. 在新模块中实现初始化函数和数据读取接口。例如：
```c:components/new_sensor/new_sensor.c
#include "new_sensor.h"

esp_err_t new_sensor_init() {
    // 传感器初始化代码
    return ESP_OK;
}

esp_err_t new_sensor_read_data(sensor_data_t *data) {
    // 数据读取代码
    return ESP_OK;
}
```
3. 在 `main.c` 中集成新传感器，调用初始化函数并在合适的位置读取数据。

#### 增加通信协议
增加通信协议需要以下步骤：
1. 在 `components` 目录下添加协议实现模块，例如 `components/new_protocol`。
2. 在 WiFi 模块中注册新的 HTTP 端点，如在 `components/wifi/wifi.c` 中添加新的请求处理函数：
```c:components/wifi/wifi.c
static esp_err_t new_protocol_handler(httpd_req_t *req) {
    // 协议处理代码
    return ESP_OK;
}

// 在合适的位置注册新端点
httpd_uri_t new_protocol_uri = {
    .uri       = "/new_protocol",
    .method    = HTTP_GET,
    .handler   = new_protocol_handler,
    .user_ctx  = NULL
};

httpd_register_uri_handler(server, &new_protocol_uri);
```
3. 添加协议解析逻辑，处理接收到的协议数据。

## 调试建议
1. 使用`ESP_LOGI`输出调试信息
2. 通过`idf.py monitor`查看实时日志
3. 使用逻辑分析仪检查PWM信号
4. 通过Postman测试WiFi API接口

## 常见问题
Q: GPS数据无法解析
A: 检查串口连接和波特率设置，确认NMEA语句格式正确

Q: IMU数据不稳定
A: 调整卡尔曼滤波参数，检查传感器安装是否牢固

Q: WiFi连接失败
A: 检查SSID/密码配置，确认信号强度足够

