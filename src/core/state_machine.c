#include "fcs/state_machine.h"
#include "fcs/fcs_math.h"

#include <math.h>
#include <string.h>

#define ARM_STICK_SECONDS   1.0f
#define DISARM_STICK_SECONDS 0.5f
#define ARMING_DELAY_US     200000u
#define LAND_DESCENT_MPS    0.6f
#define THROTTLE_STICK_DB   0.10f

static void set_state(fcs_sm_t *sm, fcs_arm_state_t s, uint64_t now_us)
{
    if (sm->arm_state == s) return;
    sm->arm_state = s;
    sm->state_enter_us = now_us;
    sm->arm_stick_ticks = 0u;
    sm->disarm_stick_ticks = 0u;
    sm->state_changes++;
}

static void latch_hold(fcs_sm_t *sm, const fcs_state_t *state)
{
    sm->hold_alt_m = state->altitude_m;
    sm->hold_pos_ned = state->pos_ned;
    sm->hold_yaw_rad = state->euler.yaw;
}

static fcs_flight_mode_t mode_from_switch(uint8_t sw)
{
    switch (sw) {
        case 0:  return FCS_MODE_STABILIZE;
        case 1:  return FCS_MODE_ALT_HOLD;
        case 2:  return FCS_MODE_POS_HOLD;
        case 3:  return FCS_MODE_AUTO;
        case 4:  return FCS_MODE_MANUAL;
        default: return FCS_MODE_STABILIZE;
    }
}

void fcs_sm_init(fcs_sm_t *sm)
{
    memset(sm, 0, sizeof(*sm));
    sm->arm_state = FCS_ARM_DISARMED;
    sm->mode = FCS_MODE_STABILIZE;
    sm->active_fault = FCS_FSAFE_NONE;
}

bool fcs_sm_prearm_check(const fcs_sm_t *sm,
                         const fcs_state_t *state,
                         const fcs_rc_t *rc,
                         const fcs_params_t *p)
{
    (void)sm;
    if (!rc->valid) return false;
    if (rc->throttle > 0.10f) return false;
    if (state->battery_v > 0.0f && state->battery_v < p->battery_min_v + 0.3f) return false;
    if (fabsf(state->euler.roll)  > 0.15f) return false;
    if (fabsf(state->euler.pitch) > 0.15f) return false;
    return true;
}

bool fcs_sm_set_waypoints(fcs_sm_t *sm, const fcs_waypoint_t *wps, uint8_t count)
{
    if (!wps || count == 0u || count > FCS_MAX_WAYPOINTS) return false;
    memcpy(sm->waypoints, wps, sizeof(fcs_waypoint_t) * count);
    sm->waypoint_count = count;
    sm->waypoint_index = 0u;
    sm->waypoint_hold_s = 0.0f;
    sm->mission_active = true;
    sm->mission_complete = false;
    return true;
}

void fcs_sm_request_mode(fcs_sm_t *sm, fcs_flight_mode_t mode)
{
    if (mode < FCS_MODE_MANUAL || mode >= FCS_MODE_COUNT) return;
    if (sm->mode == mode) return;
    sm->mode = mode;
    sm->state_changes++;
    if (mode == FCS_MODE_AUTO && sm->waypoint_count > 0u) {
        sm->waypoint_index = 0u;
        sm->waypoint_hold_s = 0.0f;
        sm->mission_active = true;
        sm->mission_complete = false;
    }
}

bool fcs_sm_armed(const fcs_sm_t *sm)
{
    return sm->arm_state == FCS_ARM_ARMED || sm->arm_state == FCS_ARM_LANDING;
}

void fcs_sm_update(fcs_sm_t *sm,
                   const fcs_rc_t *rc,
                   const fcs_state_t *state,
                   const fcs_params_t *p,
                   fcs_failsafe_reason_t fault,
                   uint64_t now_us,
                   float dt,
                   fcs_setpoint_t *sp)
{
    if (dt <= 0.0f) dt = p->loop_dt_s;
    const uint32_t arm_ticks = (uint32_t)(ARM_STICK_SECONDS / p->loop_dt_s) + 1u;
    const uint32_t disarm_ticks = (uint32_t)(DISARM_STICK_SECONDS / p->loop_dt_s) + 1u;

    /* ---------------- fault handling (highest priority) ---------------- */
    sm->active_fault = fault;
    if (fault != FCS_FSAFE_NONE) {
        if (fcs_failsafe_is_kill(fault)) {
            set_state(sm, FCS_ARM_DISARMED, now_us);
        } else if (fault == FCS_FSAFE_GNSS_LOSS && sm->mode >= FCS_MODE_POS_HOLD) {
            sm->mode = FCS_MODE_ALT_HOLD;                 /* graceful degradation */
            sm->gnss_degraded = true;
            latch_hold(sm, state);
        } else if (fault == FCS_FSAFE_RC_LOSS && sm->mode == FCS_MODE_AUTO &&
                   sm->mission_active) {
            /* An autonomous aircraft continues its mission on RC loss. */
        } else if (sm->arm_state == FCS_ARM_ARMED ||
                   sm->arm_state == FCS_ARM_ARMING) {
            latch_hold(sm, state);
            set_state(sm, FCS_ARM_LANDING, now_us);
        }
    }

    /* ---------------- stick arming / disarming ---------------- */
    bool arm_cmd    = rc->valid && rc->throttle < 0.10f && rc->yaw >  0.90f;
    bool disarm_cmd = rc->valid && rc->throttle < 0.10f && rc->yaw < -0.90f;

    switch (sm->arm_state) {
        case FCS_ARM_DISARMED:
            sm->arm_stick_ticks = arm_cmd ? sm->arm_stick_ticks + 1u : 0u;
            if (sm->arm_stick_ticks >= arm_ticks && sm->prearm_ok) {
                set_state(sm, FCS_ARM_ARMING, now_us);
            }
            break;
        case FCS_ARM_ARMING:
            if (now_us - sm->state_enter_us >= ARMING_DELAY_US) {
                latch_hold(sm, state);
                set_state(sm, FCS_ARM_ARMED, now_us);
            }
            break;
        case FCS_ARM_ARMED:
            sm->disarm_stick_ticks = disarm_cmd ? sm->disarm_stick_ticks + 1u : 0u;
            if (sm->disarm_stick_ticks >= disarm_ticks) {
                set_state(sm, FCS_ARM_DISARMED, now_us);
            }
            break;
        case FCS_ARM_LANDING:
            sm->hold_alt_m -= LAND_DESCENT_MPS * dt;
            if (sm->hold_alt_m < -0.5f) sm->hold_alt_m = -0.5f;
            if (state->altitude_m < 0.15f &&
                (now_us - sm->state_enter_us) > 1000000u) {
                set_state(sm, FCS_ARM_DISARMED, now_us);
            }
            break;
        case FCS_ARM_FAILSAFE:
            set_state(sm, FCS_ARM_LANDING, now_us);
            break;
        default:
            set_state(sm, FCS_ARM_DISARMED, now_us);
            break;
    }

    /* ---------------- mode selection ---------------- */
    if (sm->arm_state != FCS_ARM_LANDING) {
        fcs_flight_mode_t requested = mode_from_switch(rc->mode_switch);
        /* Once GNSS is lost, refuse to re-enter a position-dependent mode
         * until the core observes a fresh valid GNSS sample. */
        if (sm->gnss_degraded && requested >= FCS_MODE_POS_HOLD)
            requested = FCS_MODE_ALT_HOLD;
        if (requested != sm->mode) {
            sm->mode = requested;
            sm->state_changes++;
            if (requested == FCS_MODE_ALT_HOLD || requested == FCS_MODE_POS_HOLD)
                latch_hold(sm, state);
            if (requested == FCS_MODE_AUTO && sm->waypoint_count > 0u) {
                sm->waypoint_index = 0u;
                sm->waypoint_hold_s = 0.0f;
                sm->mission_active = true;
                sm->mission_complete = false;
            }
        }
    }

    /* ---------------- setpoint assembly ---------------- */
    sp->mode = sm->mode;
    sp->target_attitude.roll  = fcs_clampf(rc->roll,  -1.0f, 1.0f) * p->max_tilt_rad;
    sp->target_attitude.pitch = fcs_clampf(rc->pitch, -1.0f, 1.0f) * p->max_tilt_rad;
    sp->target_attitude.yaw   = sm->hold_yaw_rad;
    sp->target_yaw_rate = fcs_deadband(rc->yaw, 0.05f) * p->max_yaw_rate_rad;
    sp->target_throttle = fcs_clampf(rc->throttle, 0.0f, 1.0f);
    sp->target_alt_m    = sm->hold_alt_m;
    sp->target_pos_ned  = sm->hold_pos_ned;

    if (sm->arm_state == FCS_ARM_LANDING) {
        sp->mode = FCS_MODE_POS_HOLD;
        sp->target_pos_ned = sm->hold_pos_ned;
        sp->target_alt_m = sm->hold_alt_m;
        return;
    }

    if (sm->mode == FCS_MODE_ALT_HOLD || sm->mode == FCS_MODE_POS_HOLD) {
        /* Throttle stick above/below centre commands a climb/descent rate. */
        float stick = fcs_deadband(rc->throttle - 0.5f, THROTTLE_STICK_DB) * 2.0f;
        sm->hold_alt_m += stick * p->max_climb_rate_mps * dt;
        if (sm->hold_alt_m < 0.0f) sm->hold_alt_m = 0.0f;
        sp->target_alt_m = sm->hold_alt_m;
        sp->target_attitude.yaw = sm->hold_yaw_rad;
    }

    if (sm->mode == FCS_MODE_AUTO && sm->mission_active && sm->waypoint_count > 0u) {
        const fcs_waypoint_t *wp = &sm->waypoints[sm->waypoint_index];
        sp->target_pos_ned = wp->pos_ned;
        sp->target_alt_m = wp->alt_m;
        sp->target_attitude.yaw = wp->yaw_rad;

        float dx = wp->pos_ned.x - state->pos_ned.x;
        float dy = wp->pos_ned.y - state->pos_ned.y;
        float dz = wp->alt_m - state->altitude_m;
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        float radius = fmaxf(wp->acceptance_radius_m, p->wp_accept_radius_m);
        if (dist < radius) {
            sm->waypoint_hold_s += dt;
            if (sm->waypoint_hold_s >= wp->hold_time_s) {
                sm->waypoint_hold_s = 0.0f;
                sm->waypoint_index++;
                if (sm->waypoint_index >= sm->waypoint_count) {
                    sm->waypoint_index = sm->waypoint_count - 1u;
                    sm->mission_active = false;
                    sm->mission_complete = true;
                    latch_hold(sm, state);
                    if (sm->waypoints[sm->waypoint_index].alt_m <= 0.05f) {
                        set_state(sm, FCS_ARM_LANDING, now_us);
                    }
                }
            }
        } else {
            sm->waypoint_hold_s = 0.0f;
        }
    }
}

const char *fcs_sm_arm_name(fcs_arm_state_t s)
{
    switch (s) {
        case FCS_ARM_DISARMED: return "DISARMED";
        case FCS_ARM_ARMING:   return "ARMING";
        case FCS_ARM_ARMED:    return "ARMED";
        case FCS_ARM_LANDING:  return "LANDING";
        case FCS_ARM_FAILSAFE: return "FAILSAFE";
        default:               return "UNKNOWN";
    }
}

const char *fcs_sm_mode_name(fcs_flight_mode_t m)
{
    switch (m) {
        case FCS_MODE_MANUAL:    return "MANUAL";
        case FCS_MODE_STABILIZE: return "STABILIZE";
        case FCS_MODE_ALT_HOLD:  return "ALT_HOLD";
        case FCS_MODE_POS_HOLD:  return "POS_HOLD";
        case FCS_MODE_AUTO:      return "AUTO";
        default:                 return "UNKNOWN";
    }
}
