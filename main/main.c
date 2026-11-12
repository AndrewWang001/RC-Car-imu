#include <stdio.h>

#include "actuators.h"
#include "console.h"
#include "control.h"
#include "imu.h"
#include "params.h"
#include "rc_input.h"

void app_main(void)
{
    actuators_init();   // neutral outputs first
    params_init();
    rc_input_init();
    imu_start();
    control_start();
    console_start();

    printf("\nRC car ready, mode %s\n", params_mode_name(g_params.mode));
    printf("Keep still for gyro calibration, center throttle to arm. 'help' for commands\n\n");
}
