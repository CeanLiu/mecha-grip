idf_component_register(
    SRCS "src/complementary_filter.c" "src/my_imu.c" "src/quaternion.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
