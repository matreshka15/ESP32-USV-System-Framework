idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "."
    REQUIRES 
        freertos 
        esp_log
)