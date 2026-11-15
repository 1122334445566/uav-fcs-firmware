/**
 * @file controller.h
 * @brief Cascaded flight controller.
 *
 *   position (P+D)  -> desired horizontal acceleration
 *   acceleration    -> desired roll/pitch (tilt), with yaw rotation
 *   angle (P)       -> desired body rates
 *   rate (PID)      -> normalised torques
 *   altitude ladder -> collective thrust
 *
 * See docs/CONTROL_LAW.md for the derivation and tuning procedure.
 */
#ifndef FCS_CONTROLLER_H
#define FCS_CONTROLLER_H

#include "fcs/fcs_types.h"
#include "fcs/params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fcs_vec3_t rate_integral;
    fcs_vec3_t rate_prev;
    float      vel_z_integral;
    bool       rate_prev_valid;
    bool       initialized;
    /* Diagnostics, useful for logs and tuning */
    fcs_euler_t desired_attitude;
    fcs_vec3_t  desired_rates;
    float       desired_alt_m;
} fcs_controller_t;

void fcs_controller_init(fcs_controller_t *c);

/** Reset all integrators (call on arming / mode change). */
void fcs_controller_reset(fcs_controller_t *c);

/**
 * Run one control iteration.
 * @param state   current estimated state
 * @param sp      setpoint produced by the state machine
 * @param mode    active flight mode (MANUAL is handled by the caller)
 * @param p       parameters
 * @param dt      seconds
 */
fcs_control_t fcs_controller_update(fcs_controller_t *c,
                                    const fcs_state_t *state,
                                    const fcs_setpoint_t *sp,
                                    fcs_flight_mode_t mode,
                                    const fcs_params_t *p,
                                    float dt);

/** Convert a desired NED horizontal acceleration into roll/pitch commands. */
fcs_euler_t fcs_accel_to_tilt(fcs_vec3_t accel_ned, float yaw_rad, float max_tilt_rad);

#ifdef __cplusplus
}
#endif

#endif /* FCS_CONTROLLER_H */
