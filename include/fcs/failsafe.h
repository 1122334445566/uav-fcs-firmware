/**
 * @file failsafe.h
 * @brief Independent safety monitor. Deliberately has no dependency on the
 *        controller: it can kill the motors even if control code misbehaves.
 */
#ifndef FCS_FAILSAFE_H
#define FCS_FAILSAFE_H

#include <stdint.h>
#include "fcs/fcs_types.h"
#include "fcs/params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FCS_FSAFE_NONE = 0,
    FCS_FSAFE_RC_LOSS,
    FCS_FSAFE_BATTERY_CRITICAL,
    FCS_FSAFE_ATTITUDE_UPSET,
    FCS_FSAFE_GNSS_LOSS,
    FCS_FSAFE_IMU_ERROR,
    FCS_FSAFE_LOOP_STALL
} fcs_failsafe_reason_t;

typedef struct {
    bool      rc_valid;
    bool      imu_valid;
    bool      gnss_valid;
    bool      armed;
    float     battery_v;
    float     tilt_rad;
    float     alt_m;
    float     dt_s;
    fcs_flight_mode_t mode;
    uint64_t  now_us;
    uint64_t  last_rc_us;
} fcs_safety_input_t;

typedef struct {
    bool      active;
    fcs_failsafe_reason_t reason;
    uint32_t  imu_error_count;
    uint32_t  trigger_count;
    uint64_t  last_good_imu_us;
} fcs_failsafe_t;

void fcs_failsafe_init(fcs_failsafe_t *fs);

/**
 * Evaluate all safety rules. Returns a non-zero reason on the first tick a
 * fault appears; the caller (state machine) decides the action.
 * A returned FCS_FSAFE_ATTITUDE_UPSET / IMU_ERROR means "kill motors NOW".
 */
fcs_failsafe_reason_t fcs_failsafe_check(fcs_failsafe_t *fs,
                                         const fcs_safety_input_t *in,
                                         const fcs_params_t *p);

/** True when the fault requires an immediate motor kill. */
bool fcs_failsafe_is_kill(fcs_failsafe_reason_t reason);

void fcs_failsafe_clear(fcs_failsafe_t *fs);
const char *fcs_failsafe_name(fcs_failsafe_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* FCS_FAILSAFE_H */
