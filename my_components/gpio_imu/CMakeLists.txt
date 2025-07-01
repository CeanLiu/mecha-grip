idf_component_register(
    SRCS "quaternion.c" "complementary_filter.c" "my_imu.c"
    INCLUDE_DIRS "."           # Your own headers
    REQUIRES driver
)
