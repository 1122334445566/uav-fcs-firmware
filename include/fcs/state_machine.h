/**
 * @file state_machine.h
 * @brief Arming state machine, mode management and mission (waypoint) logic.
 *
 *   DISARMED --[stick arming + prearm OK]--> ARMING --> ARMED
 *      ^                                                |
 *      |                                    fault / disarm stick
 *      |                                                v
 *      +---------------------- LANDING <---------- FAILSAFE
 *
 * Only ARMED and LANDING may produce non-zero motor output. The core enforces
 * this a second time, so a bug in the controller cannot spin the props.
 */
#ifndef FCS_STATE_MACHINE_H
#define FCS_STATE_MACHINE_H

#include "fcs/fcs_types.h"
#include "fcs/failsafe.h"
#include "fcs/params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fcs_arm_state_t   arm_state;
    fcs_flight_mode_t mode;
    uint64_t          state_enter_us;
    uint32_t          arm_stick_ticks;
    uint32_t          disarm_stick_ticks;

    float             hold_alt_m;
    fcs_vec3_t        hold_pos_ned;
    float             hold_yaw_rad;

    fcs_waypoint_t    waypoints[FCS_MAX_WAYPOINTS];
    uint8_t           waypoint_count;
    uint8_t           waypoint_index;
    float             waypoint_hold_s;
    bool              mission_active;
    bool              mission_complete;

    fcs_failsafe_reason_t active_fault;
    bool              prearm_ok;
    bool              gnss_degraded;   /**< block POS_HOLD/AUTO until GNSS returns */
    uint32_t          state_changes;
} fcs_sm_t;

void fcs_sm_init(fcs_sm_t *sm);

/** Pre-arm checks: level attitude, IMU healthy, battery, valid RC, low throttle. */
bool fcs_sm_prearm_check(const fcs_sm_t *sm,
                         const fcs_state_t *state,
                         const fcs_rc_t *rc,
                         const fcs_params_t *p);

/** Install a mission (copies at most FCS_MAX_WAYPOINTS items). */
bool fcs_sm_set_waypoints(fcs_sm_t *sm, const fcs_waypoint_t *wps, uint8_t count);

/** Ground-station override of the flight mode. */
void fcs_sm_request_mode(fcs_sm_t *sm, fcs_flight_mode_t mode);

/**
 * Advance the machine by one tick and produce the setpoint for the controller.
 * @param fault  reason reported by fcs_failsafe_check() this tick (NONE = healthy)
 */
void fcs_sm_update(fcs_sm_t *sm,
                   const fcs_rc_t *rc,
                   const fcs_state_t *state,
                   const fcs_params_t *p,
                   fcs_failsafe_reason_t fault,
                   uint64_t now_us,
                   float dt,
                   fcs_setpoint_t *sp);

bool fcs_sm_armed(const fcs_sm_t *sm);
const char *fcs_sm_arm_name(fcs_arm_state_t s);
const char *fcs_sm_mode_name(fcs_flight_mode_t m);

#ifdef __cplusplus
}
#endif

#endif /* FCS_STATE_MACHINE_H */
