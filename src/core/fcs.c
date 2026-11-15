#include "fcs/fcs.h"
#include "fcs/fcs_math.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Build with -DFCS_COURSE_AIDING=1 to enable (see the note in fcs_step). */
#ifndef FCS_COURSE_AIDING
#define FCS_COURSE_AIDING 0
#endif

bool fcs_init(fcs_t *f, const fcs_hal_t *hal, const fcs_params_t *user_params)
{
    if (!f || !fcs_hal_validate(hal)) return false;
    memset(f, 0, sizeof(*f));
    f->hal = *hal;

    if (user_params) {
        f->params = *user_params;
        (void)fcs_params_sanitize(&f->params);
    } else {
        fcs_params_load(&f->params, &f->hal);
    }

    fcs_attitude_init(&f->attitude, &f->params);
    fcs_controller_init(&f->controller);
    fcs_sm_init(&f->sm);
    fcs_failsafe_init(&f->failsafe);
    fcs_telemetry_init(&f->telemetry);

    f->state.quat = fcs_quat_identity();
    f->state.battery_v = 0.0f;
    f->telemetry_divider = 50u;      /* 10 Hz at a 500 Hz main loop */
    f->start_us = f->hal.time_us(f->hal.ctx);
    f->last_tick_us = 0u;
    f->last_rc_us = f->start_us;
    f->control.thrust = 0.0f;
    f->ready = true;
    return true;
}

void fcs_log(fcs_t *f, fcs_log_level_t level, const char *fmt, ...)
{
    if (!f || !f->hal.log) return;
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    f->hal.log(level, buf, f->hal.ctx);
}

float fcs_step(fcs_t *f, uint64_t now_us)
{
    if (!f || !f->ready) return 0.0f;
    void *ctx = f->hal.ctx;

    /* ---------------- timing ---------------- */
    uint64_t nominal_us = (uint64_t)(f->params.loop_dt_s * 1000000.0f);
    uint64_t delta_us = (f->last_tick_us == 0u || now_us <= f->last_tick_us)
                      ? nominal_us : (now_us - f->last_tick_us);
    float dt = (float)delta_us * 1e-6f;
    if (dt < 0.0002f) dt = 0.0002f;
    if (dt > 0.05f)   dt = 0.05f;
    f->last_tick_us = now_us;
    f->dt = dt;
    f->last_loop_us = (uint32_t)delta_us;
    if (delta_us > f->max_loop_us) f->max_loop_us = (uint32_t)delta_us;

    /* ---------------- sensors ---------------- */
    f->imu_ok = f->hal.imu_read(&f->imu, ctx) ? true : false;

    if (f->hal.baro_read && f->hal.baro_read(&f->baro, ctx) && f->baro.valid) {
        f->state.altitude_m = f->baro.altitude_m;
        f->state.vel_ned.z = -f->baro.climb_rate_mps;
    }
    bool gnss_ok = f->hal.gnss_read &&
                   f->hal.gnss_read(&f->gnss, ctx) && f->gnss.valid;
    if (gnss_ok) {
        f->sm.gnss_degraded = false;
        f->state.pos_ned.x = f->gnss.pos_n_m;
        f->state.pos_ned.y = f->gnss.pos_e_m;
        f->state.pos_ned.z = -f->state.altitude_m;
        f->state.vel_ned.x = f->gnss.vel_n_mps;
        f->state.vel_ned.y = f->gnss.vel_e_mps;
        if (f->gnss.fix_type >= 3u) f->state.vel_ned.z = f->gnss.vel_d_mps;
        f->state.position_valid = true;
    } else {
        f->state.position_valid = false;
    }
    if (f->hal.rc_read(&f->rc, ctx) && f->rc.valid) f->last_rc_us = now_us;
    if (f->hal.battery_read) {
        float v = 0.0f;
        if (f->hal.battery_read(&v, ctx)) f->state.battery_v = v;
    }

    /* ---------------- state estimation ---------------- */
    if (f->imu_ok) {
        (void)fcs_attitude_update(&f->attitude, &f->imu, &f->params, dt);
#if FCS_COURSE_AIDING
        /* Optional course-over-ground yaw aiding. DISABLED by default:
         * course equals heading only in still air with no sideslip, and
         * feeding it back while the position loop is braking creates a
         * positive feedback loop (heading follows velocity, velocity follows
         * heading). Enable only with dual-antenna GNSS or a validated
         * sideslip estimate, and keep the gain very small. */
        if (f->gnss.valid && f->params.est_kmag <= 0.0f) {
            float vn = f->gnss.vel_n_mps, ve = f->gnss.vel_e_mps;
            if (sqrtf(vn*vn + ve*ve) > 2.0f) {
                fcs_attitude_fuse_heading(&f->attitude, atan2f(ve, vn), 0.0001f);
            }
        }
#endif
    }
    f->state.quat  = f->attitude.quat;
    f->state.euler = fcs_quat_to_euler(f->state.quat);
    f->state.rates = fcs_attitude_rates(&f->imu, &f->attitude);

    /* ---------------- safety ---------------- */
    fcs_safety_input_t safety_in;
    memset(&safety_in, 0, sizeof(safety_in));
    safety_in.rc_valid   = f->rc.valid;
    safety_in.imu_valid  = f->imu_ok;
    safety_in.gnss_valid = gnss_ok;
    safety_in.armed      = fcs_sm_armed(&f->sm);
    safety_in.battery_v  = f->state.battery_v;
    safety_in.tilt_rad   = sqrtf(f->state.euler.roll  * f->state.euler.roll +
                                 f->state.euler.pitch * f->state.euler.pitch);
    safety_in.alt_m      = f->state.altitude_m;
    safety_in.dt_s       = dt;
    safety_in.mode       = f->sm.mode;
    safety_in.now_us     = now_us;
    safety_in.last_rc_us = f->last_rc_us;

    fcs_failsafe_reason_t fault =
        fcs_failsafe_check(&f->failsafe, &safety_in, &f->params);
    if (fault != FCS_FSAFE_NONE)
        fcs_log(f, FCS_LOG_WARN, "failsafe: %s", fcs_failsafe_name(fault));

    f->sm.prearm_ok = fcs_sm_prearm_check(&f->sm, &f->state, &f->rc, &f->params)
                      && f->imu_ok;

    fcs_arm_state_t   old_arm  = f->sm.arm_state;
    fcs_flight_mode_t old_mode = f->sm.mode;
    fcs_sm_update(&f->sm, &f->rc, &f->state, &f->params,
                  fault, now_us, dt, &f->setpoint);

    if (old_arm != f->sm.arm_state) {
        fcs_log(f, FCS_LOG_INFO, "arm state: %s -> %s",
                fcs_sm_arm_name(old_arm), fcs_sm_arm_name(f->sm.arm_state));
        fcs_controller_reset(&f->controller);
    }
    if (old_mode != f->sm.mode) {
        fcs_log(f, FCS_LOG_INFO, "mode: %s -> %s",
                fcs_sm_mode_name(old_mode), fcs_sm_mode_name(f->sm.mode));
        fcs_controller_reset(&f->controller);
    }

    /* ---------------- control ---------------- */
    bool enabled = (f->sm.arm_state == FCS_ARM_ARMED ||
                    f->sm.arm_state == FCS_ARM_LANDING);
    if (enabled) {
        if (f->sm.mode == FCS_MODE_MANUAL && f->sm.arm_state == FCS_ARM_ARMED) {
            f->control.thrust = fcs_clampf(f->rc.throttle,
                                           f->params.throttle_min,
                                           f->params.throttle_max);
            f->control.torque = fcs_vec3(f->rc.roll, f->rc.pitch, f->rc.yaw);
        } else {
            f->control = fcs_controller_update(&f->controller,
                                               &f->state, &f->setpoint,
                                               f->setpoint.mode,
                                               &f->params, dt);
        }
        fcs_mixer_mix(&f->control, &f->params, f->motors, NULL);
    }
    /* Independent motor kill: even a broken controller cannot spin props. */
    if (!enabled) {
        fcs_mixer_disarm(f->motors, FCS_MIXER_XQUAD_MOTORS);
        f->control.thrust = 0.0f;
        f->control.torque = fcs_vec3(0.0f, 0.0f, 0.0f);
    }
    f->hal.motor_write(f->motors, FCS_MIXER_XQUAD_MOTORS, ctx);

    /* ---------------- telemetry & watchdog ---------------- */
    f->loop_count++;
    if (f->hal.telemetry_write && f->telemetry_divider > 0u &&
        (f->loop_count % f->telemetry_divider) == 0u) {
        uint32_t uptime_ms = (uint32_t)((now_us - f->start_us) / 1000u);
        fcs_telemetry_send_heartbeat(&f->telemetry, &f->hal, uptime_ms,
                                     f->sm.arm_state, f->sm.mode,
                                     (uint8_t)f->failsafe.reason,
                                     f->last_loop_us);
        fcs_telemetry_send_attitude(&f->telemetry, &f->hal, &f->state);
        fcs_telemetry_send_position(&f->telemetry, &f->hal, &f->state);
        fcs_telemetry_send_actuators(&f->telemetry, &f->hal,
                                     f->motors, FCS_MIXER_XQUAD_MOTORS,
                                     f->control.thrust);
    }
    if (f->hal.watchdog_feed) f->hal.watchdog_feed(ctx);
    return dt;
}

bool fcs_set_waypoints(fcs_t *f, const fcs_waypoint_t *wps, uint8_t count)
{
    if (!f) return false;
    return fcs_sm_set_waypoints(&f->sm, wps, count);
}

void fcs_request_mode(fcs_t *f, fcs_flight_mode_t mode)
{
    if (f) fcs_sm_request_mode(&f->sm, mode);
}

const fcs_state_t *fcs_state(const fcs_t *f)
{ return f ? &f->state : NULL; }

const fcs_sm_t *fcs_state_machine(const fcs_t *f)
{ return f ? &f->sm : NULL; }
