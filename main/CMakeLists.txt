idf_component_register(
    SRCS "main.c" "actuators.c" "rc_input.c" "imu.c" "pid.c" "control.c" "params.c" "console.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart esp_driver_i2c esp_timer nvs_flash)
