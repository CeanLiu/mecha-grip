idf_component_register(
    SRCS "my_servo.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc
)
