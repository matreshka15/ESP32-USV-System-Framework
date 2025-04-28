idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "."
    REQUIRES 
        freertos 
        esp_log 
        driver/i2c
)