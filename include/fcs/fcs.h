/**
 * @file fcs.h
 * @brief Top level flight-core scheduler.
 *
 * Typical MCU main loop (1 kHz timer interrupt or RTOS task):
 *
 *   fcs_t fcs;
 *   fcs_init(&fcs, &board_hal, NULL);
 *   for (;;) {
 *       uint64_t now = hal.time_us(hal.ctx);
 *       if (now - last >= 1000) { fcs_step(&fcs, now); last = now; }
 *   }
 *
 * fcs_step() is deterministic: no heap allocation, no blocking calls, all
 * state lives in the caller-provided fcs_t. It is therefore safe to run the
 * exact same binary against a SIL simulator (platform/posix).
 */
#ifndef FCS_H
#define FCS_H

#include <stdarg.h>
#include "fcs/fcs_types.h"
#include "fcs/hal.h"
#include "fcs/params.h"
#include "fcs/attitude.h"
#include "fcs/controller.h"
#include "fcs/mixer.h"
#include "fcs/state_machine.h"
#include "fcs/failsafe.h"
#include "fcs/telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fcs_hal_t         hal;
    fcs_params_t      params;
    fcs_attitude_t    attitude;
    fcs_controller_t  controller;
    fcs_sm_t          sm;
    fcs_failsafe_t    failsafe;
    fcs_telemetry_t   telemetry;

    fcs_state_t       state;
    fcs_setpoint_t    setpoint;
    fcs_control_t     control;
    float             motors[FCS_MIXER_XQUAD_MOTORS];

    fcs_imu_t         imu;
    fcs_baro_t        baro;
    fcs_gnss_t        gnss;
    fcs_rc_t          rc;

    uint64_t          last_tick_us;
    uint64_t          start_us;
    uint64_t          last_rc_us;
    uint32_t          loop_count;
    uint32_t          max_loop_us;
    uint32_t          last_loop_us;
    uint32_t          telemetry_divider;
    float             dt;
    bool              imu_ok;
    bool              ready;
} fcs_t;

/**
 * Initialise the core.
 * @param hal          mandatory, must pass fcs_hal_validate()
 * @param user_params  optional; NULL -> defaults (and HAL flash load attempt)
 * @return true on success
 */
bool fcs_init(fcs_t *fcs, const fcs_hal_t *hal, const fcs_params_t *user_params);

/** Execute one control cycle. now_us must be monotonic. Returns the loop dt in s. */
float fcs_step(fcs_t *fcs, uint64_t now_us);

/** Install a waypoint mission from a ground station / test harness. */
bool fcs_set_waypoints(fcs_t *fcs, const fcs_waypoint_t *wps, uint8_t count);

void fcs_request_mode(fcs_t *fcs, fcs_flight_mode_t mode);

/** printf-style logging through the HAL. */
void fcs_log(fcs_t *fcs, fcs_log_level_t level, const char *fmt, ...);

const fcs_state_t *fcs_state(const fcs_t *fcs);
const fcs_sm_t    *fcs_state_machine(const fcs_t *fcs);

#ifdef __cplusplus
}
#endif

#endif /* FCS_H */
