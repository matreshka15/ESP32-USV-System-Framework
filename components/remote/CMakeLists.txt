idf_component_register(
    SRCS "remote.c"
    INCLUDE_DIRS "."
    REQUIRES 
        freertos 
        esp_log 
        driver/pcnt
)