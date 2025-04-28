idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        freertos 
        esp_log 
        gps 
        imu 
        motor 
        navigation 
        wifi 
        remote
)