idf_component_register(
    SRCS "navigation.c"
    INCLUDE_DIRS "."
    REQUIRES 
        freertos 
        esp_log
)