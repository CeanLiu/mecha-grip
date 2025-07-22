idf_component_register(SRCS "mecha-grip.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES gpio_servo gpio_imu my_wifi nvs_flash)
