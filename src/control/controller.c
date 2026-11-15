#include "fcs/controller.h"
#include "fcs/fcs_math.h"

#include <math.h>
#include <string.h>

void fcs_controller_init(fcs_controller_t *c)
{
    memset(c, 0, sizeof(*c));
    c->initialized = true;
}

void fcs_controller_reset(fcs_controller_t *c)
{
    c->rate_integral = fcs_vec3(0.0f, 0.0f, 0.0f);
    c->vel_z_integral = 0.0f;
    c->rate_prev_valid = false;
}

fcs_euler_t fcs_accel_to_tilt(fcs_vec3_t a, float yaw_rad, float max_tilt_rad)
{
    float c = cosf(yaw_rad);
    float s = sinf(yaw_rad);
    fcs_euler_t e;
    /* Small-angle inverse of the thrust vector, see docs/CONTROL_LAW.md.
     * Positive pitch tips the nose DOWN -> accelerates towards South (-N). */
    e.pitch = -(a.x * c + a.y * s) / FCS_GRAVITY_MPS2;
    e.roll  =  (a.y * c - a.x * s) / FCS_GRAVITY_MPS2;
    e.yaw   = 0.0f;

    float mag = sqrtf(e.roll*e.roll + e.pitch*e.pitch);
    if (mag > max_tilt_rad && mag > 1e-6f) {
        float k = max_tilt_rad / mag;
        e.roll *= k;
        e.pitch *= k;
    }
    e.roll  = fcs_clampf(e.roll,  -max_tilt_rad, max_tilt_rad);
    e.pitch = fcs_clampf(e.pitch, -max_tilt_rad, max_tilt_rad);
    return e;
}

fcs_control_t fcs_controller_update(fcs_controller_t *c,
                                    const fcs_state_t *state,
                                    const fcs_setpoint_t *sp,
                                    fcs_flight_mode_t mode,
                                    const fcs_params_t *p,
                                    float dt)
{
    fcs_control_t out;
    memset(&out, 0, sizeof(out));
    if (!c->initialized || dt <= 0.0f) return out;

    /* ---------- 1. desired attitude ---------- */
    fcs_euler_t att_sp = sp->target_attitude;

    if (mode == FCS_MODE_POS_HOLD || mode == FCS_MODE_AUTO) {
        fcs_vec3_t a;
        a.x = p->pos_xy_kp * (sp->target_pos_ned.x - state->pos_ned.x)
            - p->pos_xy_kd * state->vel_ned.x;
        a.y = p->pos_xy_kp * (sp->target_pos_ned.y - state->pos_ned.y)
            - p->pos_xy_kd * state->vel_ned.y;
        a.z = 0.0f;
        float max_a = FCS_GRAVITY_MPS2 * tanf(p->max_tilt_rad);
        float mag = sqrtf(a.x*a.x + a.y*a.y);
        if (mag > max_a && mag > 1e-6f) {
            a.x *= max_a / mag;
            a.y *= max_a / mag;
        }
        fcs_euler_t tilt = fcs_accel_to_tilt(a, state->euler.yaw, p->max_tilt_rad);
        att_sp.roll  = tilt.roll;
        att_sp.pitch = tilt.pitch;
    }
    c->desired_attitude = att_sp;

    /* ---------- 2. angle (P) loop -> body rate setpoint ---------- */
    fcs_vec3_t rate_sp;
    rate_sp.x = fcs_clampf(p->att_kp_roll  * fcs_wrap_pi(att_sp.roll  - state->euler.roll),
                           -p->max_rate_rad, p->max_rate_rad);
    rate_sp.y = fcs_clampf(p->att_kp_pitch * fcs_wrap_pi(att_sp.pitch - state->euler.pitch),
                           -p->max_rate_rad, p->max_rate_rad);
    if (mode <= FCS_MODE_ALT_HOLD) {
        rate_sp.z = fcs_clampf(sp->target_yaw_rate, -p->max_yaw_rate_rad, p->max_yaw_rate_rad);
    } else {
        rate_sp.z = fcs_clampf(p->att_kp_yaw * fcs_wrap_pi(att_sp.yaw - state->euler.yaw),
                               -p->max_yaw_rate_rad, p->max_yaw_rate_rad);
    }
    c->desired_rates = rate_sp;

    /* ---------- 3. rate (PID) loop -> normalised torques ---------- */
    float err[3]   = { rate_sp.x - state->rates.x,
                       rate_sp.y - state->rates.y,
                       rate_sp.z - state->rates.z };
    float rate[3]  = { state->rates.x, state->rates.y, state->rates.z };
    float integ[3] = { c->rate_integral.x, c->rate_integral.y, c->rate_integral.z };
    float torque[3];

    for (int i = 0; i < 3; ++i) {
        integ[i] = fcs_clampf(integ[i] + err[i] * dt,
                              -p->rate_ilimit[i], p->rate_ilimit[i]);
        /* Derivative on measurement: no setpoint kick on stick moves. */
        float dmeas = 0.0f;
        if (c->rate_prev_valid && dt > 0.0f) {
            float prev = (i == 0) ? c->rate_prev.x :
                         ((i == 1) ? c->rate_prev.y : c->rate_prev.z);
            dmeas = (rate[i] - prev) / dt;
        }
        torque[i] = p->rate_kp[i] * err[i]
                  + p->rate_ki[i] * integ[i]
                  - p->rate_kd[i] * dmeas;
        torque[i] = fcs_clampf(torque[i], -1.0f, 1.0f);
    }
    c->rate_integral = fcs_vec3(integ[0], integ[1], integ[2]);
    c->rate_prev = state->rates;
    c->rate_prev_valid = true;

    /* ---------- 4. collective thrust ---------- */
    float thrust;
    if (mode == FCS_MODE_STABILIZE || mode == FCS_MODE_MANUAL) {
        thrust = sp->target_throttle;
        c->vel_z_integral = 0.0f;
    } else {
        float alt_err = sp->target_alt_m - state->altitude_m;
        float vz_sp   = fcs_clampf(p->alt_kp * alt_err,
                                   -p->max_climb_rate_mps, p->max_climb_rate_mps);
        float vz      = -state->vel_ned.z;          /* NED down -> up positive */
        float vz_err  = vz_sp - vz;
        c->vel_z_integral = fcs_clampf(c->vel_z_integral + vz_err * dt,
                                       -p->vel_z_ilimit, p->vel_z_ilimit);
        thrust = p->hover_throttle
               + p->vel_z_kp * vz_err
               + p->vel_z_ki * c->vel_z_integral;
    }
    c->desired_alt_m = sp->target_alt_m;

    out.thrust = fcs_clampf(thrust, p->throttle_min, p->throttle_max);
    out.torque = fcs_vec3(torque[0], torque[1], torque[2]);
    return out;
}
