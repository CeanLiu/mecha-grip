idf_component_register(
    SRCS "my_imu.c"
    INCLUDE_DIRS "."           # Your own headers
    REQUIRES driver espressif__mpu6050  # Link to the managed component by folder name
)
