idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos log
    PRIV_REQUIRES led_strip
)
