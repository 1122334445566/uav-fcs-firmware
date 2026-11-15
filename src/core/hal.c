#include "fcs/hal.h"

bool fcs_hal_validate(const fcs_hal_t *hal)
{
    if (!hal) return false;
    return hal->imu_read != 0
        && hal->rc_read != 0
        && hal->motor_write != 0
        && hal->time_us != 0;
}
