#include "fcs/failsafe.h"
#include "fcs/fcs_math.h"

void fcs_failsafe_init(fcs_failsafe_t *fs)
{
    fs->active = false;
    fs->reason = FCS_FSAFE_NONE;
    fs->imu_error_count = 0u;
    fs->trigger_count = 0u;
    fs->last_good_imu_us = 0u;
}

static fcs_failsafe_reason_t evaluate(fcs_failsafe_t *fs,
                                      const fcs_safety_input_t *in,
                                      const fcs_params_t *p)
{
    /* Loop stall: the scheduler stopped being called in time. */
    if (in->dt_s * 1000.0f > p->loop_timeout_ms) return FCS_FSAFE_LOOP_STALL;

    if (in->imu_valid) {
        fs->imu_error_count = 0u;
        fs->last_good_imu_us = in->now_us;
    } else {
        fs->imu_error_count++;
        if (fs->imu_error_count >= p->imu_error_limit) return FCS_FSAFE_IMU_ERROR;
    }

    if (!in->armed) return FCS_FSAFE_NONE;

    if (in->tilt_rad > p->fault_tilt_rad) return FCS_FSAFE_ATTITUDE_UPSET;

    uint64_t rc_age_us = (in->now_us > in->last_rc_us)
                       ? (in->now_us - in->last_rc_us) : 0u;
    if (!in->rc_valid || rc_age_us > (uint64_t)(p->rc_timeout_ms * 1000.0f))
        return FCS_FSAFE_RC_LOSS;

    if (in->battery_v > 0.0f && in->battery_v < p->battery_min_v)
        return FCS_FSAFE_BATTERY_CRITICAL;

    if (!in->gnss_valid &&
        (in->mode == FCS_MODE_POS_HOLD || in->mode == FCS_MODE_AUTO))
        return FCS_FSAFE_GNSS_LOSS;

    return FCS_FSAFE_NONE;
}

fcs_failsafe_reason_t fcs_failsafe_check(fcs_failsafe_t *fs,
                                         const fcs_safety_input_t *in,
                                         const fcs_params_t *p)
{
    fcs_failsafe_reason_t r = evaluate(fs, in, p);

    if (r == FCS_FSAFE_NONE) {
        if (fs->active) {                 /* fault cleared */
            fs->active = false;
            fs->reason = FCS_FSAFE_NONE;
        }
        return FCS_FSAFE_NONE;
    }
    if (!fs->active || fs->reason != r) {
        fs->active = true;
        fs->reason = r;
        fs->trigger_count++;
        return r;                          /* emit the event only once */
    }
    return FCS_FSAFE_NONE;                 /* same fault still ongoing */
}

bool fcs_failsafe_is_kill(fcs_failsafe_reason_t reason)
{
    return reason == FCS_FSAFE_ATTITUDE_UPSET
        || reason == FCS_FSAFE_IMU_ERROR
        || reason == FCS_FSAFE_LOOP_STALL;
}

void fcs_failsafe_clear(fcs_failsafe_t *fs)
{
    fcs_failsafe_init(fs);
}

const char *fcs_failsafe_name(fcs_failsafe_reason_t reason)
{
    switch (reason) {
        case FCS_FSAFE_NONE:               return "NONE";
        case FCS_FSAFE_RC_LOSS:           return "RC_LOSS";
        case FCS_FSAFE_BATTERY_CRITICAL:  return "BATTERY_CRITICAL";
        case FCS_FSAFE_ATTITUDE_UPSET:    return "ATTITUDE_UPSET";
        case FCS_FSAFE_GNSS_LOSS:         return "GNSS_LOSS";
        case FCS_FSAFE_IMU_ERROR:         return "IMU_ERROR";
        case FCS_FSAFE_LOOP_STALL:        return "LOOP_STALL";
        default:                          return "UNKNOWN";
    }
}
