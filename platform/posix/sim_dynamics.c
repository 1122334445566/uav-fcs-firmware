#include "sim.h"
#include "fcs/fcs_math.h"
#include "fcs/mixer.h"

#include <math.h>
#include <string.h>

/** Deterministic small xorshift32 PRNG -> uniform [-1, 1]. */
static float noise(fcs_sim_t *s)
{
    uint32_t x = s->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s->rng_state = x ? x : 0x9E3779B9u;
    return ((float)(x % 2000001u) / 1000000.0f) - 1.0f;
}

void fcs_sim_init(fcs_sim_t *s, uint32_t seed)
{
    memset(s, 0, sizeof(*s));
    s->quat = fcs_quat_identity();
    s->pos_ned = fcs_vec3(0.0f, 0.0f, 0.0f);
    s->vel_ned = fcs_vec3(0.0f, 0.0f, 0.0f);
    s->rates   = fcs_vec3(0.0f, 0.0f, 0.0f);
    /* At rest on the ground the kinematic acceleration is zero; the
     * accelerometer therefore measures the 1 g support force (upwards). */
    s->accel_ned = fcs_vec3(0.0f, 0.0f, 0.0f);

    s->mass_kg = 1.2f;
    s->ixx = 0.015f; s->iyy = 0.015f; s->izz = 0.030f;
    /* 4 motors * 0.5 (hover) * thrust_max_n == m*g */
    s->thrust_max_n = s->mass_kg * FCS_GRAVITY_MPS2 / 2.0f;
    s->torque_roll_pitch = 2.0f;
    s->torque_yaw = 0.5f;
    s->motor_tau_s = 0.020f;
    s->drag_xy = 0.20f;

    s->battery_v = 16.0f;
    s->gnss_available = true;
    s->rc_available = true;
    s->imu_available = true;
    s->rng_state = seed ? seed : 0x1234ABCDu;
    s->verbose = false;
}

void fcs_sim_write_motors(fcs_sim_t *s, const float *outputs, uint8_t count)
{
    if (count > 4u) count = 4u;
    for (uint8_t i = 0; i < count; ++i) s->motor_cmd[i] = outputs[i];
    s->motor_writes++;
}

void fcs_sim_step(fcs_sim_t *s, float dt)
{
    if (dt <= 0.0f) return;

    /* ESC + propeller first-order lag. */
    float k = fcs_clampf(dt / s->motor_tau_s, 0.0f, 1.0f);
    for (uint8_t i = 0; i < 4u; ++i)
        s->motors[i] += (s->motor_cmd[i] - s->motors[i]) * k;

    /* Translational dynamics: thrust along the body -z axis + gravity + drag. */
    float total = s->motors[0] + s->motors[1] + s->motors[2] + s->motors[3];
    fcs_vec3_t up_ned = fcs_quat_rotate(s->quat, fcs_vec3(0.0f, 0.0f, -1.0f));
    float a_thrust = total * s->thrust_max_n / s->mass_kg;
    fcs_vec3_t accel = fcs_vec3_add(
        fcs_vec3_scale(up_ned, a_thrust),
        fcs_vec3(0.0f, 0.0f, FCS_GRAVITY_MPS2));
    accel.x -= s->drag_xy * s->vel_ned.x;
    accel.y -= s->drag_xy * s->vel_ned.y;
    accel.z -= 0.10f * s->vel_ned.z;

    /* While resting on the ground the normal force cancels any downward
     * acceleration. Without this the simulated accelerometer would read 0 g
     * on the pad, which is physically wrong (it measures the support force). */
    if (s->pos_ned.z >= 0.0f && accel.z > 0.0f) accel.z = 0.0f;
    s->accel_ned = accel;

    /* Rotational dynamics. The torque sign convention is shared with mixer.c
     * so that the simulated plant and the control allocation agree. */
    fcs_vec3_t t = fcs_mixer_torque_from_outputs(s->motors);
    float mx = t.x * s->torque_roll_pitch;
    float my = t.y * s->torque_roll_pitch;
    float mz = t.z * s->torque_yaw;
    fcs_vec3_t alpha;
    alpha.x = (mx - (s->izz - s->iyy) * s->rates.y * s->rates.z) / s->ixx
              - 0.6f * s->rates.x;
    alpha.y = (my - (s->ixx - s->izz) * s->rates.x * s->rates.z) / s->iyy
              - 0.6f * s->rates.y;
    alpha.z = (mz - (s->iyy - s->ixx) * s->rates.x * s->rates.y) / s->izz
              - 0.4f * s->rates.z;

    s->rates = fcs_vec3_add(s->rates, fcs_vec3_scale(alpha, dt));
    s->quat  = fcs_quat_integrate(s->quat, s->rates, dt);
    s->vel_ned = fcs_vec3_add(s->vel_ned, fcs_vec3_scale(accel, dt));
    s->pos_ned = fcs_vec3_add(s->pos_ned, fcs_vec3_scale(s->vel_ned, dt));

    /* Ground plane at NED z = 0 (positive z is below ground). */
    if (s->pos_ned.z > 0.0f) {
        s->pos_ned.z = 0.0f;
        if (s->vel_ned.z > 0.0f) s->vel_ned.z = 0.0f;
        s->vel_ned.x *= 0.98f;
        s->vel_ned.y *= 0.98f;
    }
}

void fcs_sim_sample_imu(fcs_sim_t *s, fcs_imu_t *out)
{
    memset(out, 0, sizeof(*out));
    /* Specific force = kinematic acceleration - gravity, expressed in body. */
    fcs_vec3_t sf_ned = fcs_vec3_sub(s->accel_ned,
                                     fcs_vec3(0.0f, 0.0f, FCS_GRAVITY_MPS2));
    fcs_vec3_t sf_body = fcs_quat_rotate_inv(s->quat, sf_ned);

    static const fcs_vec3_t gyro_bias = { 0.010f, -0.008f, 0.015f };
    out->gyro = fcs_vec3_add(
        fcs_vec3_add(s->rates, gyro_bias),
        fcs_vec3(0.004f * noise(s), 0.004f * noise(s), 0.004f * noise(s)));
    out->accel = fcs_vec3_add(sf_body,
        fcs_vec3(0.05f * noise(s), 0.05f * noise(s), 0.05f * noise(s)));
    /* Geomagnetic field in NED: north + down (northern hemisphere). */
    fcs_vec3_t mag_body = fcs_quat_rotate_inv(s->quat,
                              fcs_vec3(0.35f, 0.0f, 0.60f));
    out->mag = fcs_vec3_add(mag_body,
        fcs_vec3(0.01f * noise(s), 0.01f * noise(s), 0.01f * noise(s)));
    out->timestamp_us = s->time_us;
    out->accel_valid = true;
    out->mag_valid = true;
}

void fcs_sim_sample_baro(fcs_sim_t *s, fcs_baro_t *out)
{
    memset(out, 0, sizeof(*out));
    out->altitude_m = -s->pos_ned.z + 0.02f * noise(s);
    out->climb_rate_mps = -s->vel_ned.z + 0.05f * noise(s);
    out->temperature_c = 25.0f;
    out->valid = true;
}

void fcs_sim_sample_gnss(fcs_sim_t *s, fcs_gnss_t *out)
{
    memset(out, 0, sizeof(*out));
    out->pos_n_m = s->pos_ned.x + 0.05f * noise(s);
    out->pos_e_m = s->pos_ned.y + 0.05f * noise(s);
    out->vel_n_mps = s->vel_ned.x + 0.02f * noise(s);
    out->vel_e_mps = s->vel_ned.y + 0.02f * noise(s);
    out->vel_d_mps = s->vel_ned.z + 0.02f * noise(s);
    out->alt_msl_m = -s->pos_ned.z;
    out->lat_deg = 22.5;
    out->lon_deg = 114.0;
    out->fix_type = 3u;
    out->satellites = 14u;
    out->valid = s->gnss_available;
}

void fcs_sim_update_rc(fcs_sim_t *s, fcs_sim_scenario_t scenario, float t_s)
{
    s->battery_v = 16.0f - 0.002f * t_s;

    if (scenario == FCS_SIM_FAILSAFE && t_s >= 12.0f) {
        s->battery_v = 12.0f;          /* critical under-voltage */
        s->rc_available = false;       /* simultaneously lose the RC link */
    } else {
        s->rc_available = true;
    }

    s->rc.valid = s->rc_available;
    s->rc.last_update_us = s->time_us;
    s->rc.mode_switch = 3u;            /* AUTO */
    s->rc.arm_switch = true;
    s->rc.roll = 0.0f;
    s->rc.pitch = 0.0f;

    if (t_s < 1.5f) {
        /* Classic stick arming: throttle low + full right yaw for 1 s. */
        s->rc.throttle = 0.0f;
        s->rc.yaw = 1.0f;
    } else {
        s->rc.throttle = 0.5f;
        s->rc.yaw = 0.0f;
    }
}

uint8_t fcs_sim_mission(fcs_sim_scenario_t scenario, fcs_waypoint_t *out)
{
    static const fcs_waypoint_t hover_mission[] = {
        { {0.0f, 0.0f, -5.0f}, 5.0f, 0.0f, 1.0f, 3.0f }
    };
    static const fcs_waypoint_t wp_mission[] = {
        { { 0.0f, 0.0f, -5.0f}, 5.0f, 0.0f,        1.2f, 1.0f },
        { { 8.0f, 4.0f, -5.0f}, 5.0f, 0.0f,        1.2f, 1.0f },
        { { 8.0f, 4.0f, -8.0f}, 8.0f, 0.78539816f, 1.2f, 1.0f },
        { { 0.0f, 0.0f, -5.0f}, 5.0f, 0.0f,        1.2f, 1.0f },
        { { 0.0f, 0.0f,  0.0f}, 0.0f, 0.0f,        1.2f, 0.0f }
    };

    if (scenario == FCS_SIM_WAYPOINT) {
        memcpy(out, wp_mission, sizeof(wp_mission));
        return (uint8_t)(sizeof(wp_mission) / sizeof(wp_mission[0]));
    }
    memcpy(out, hover_mission, sizeof(hover_mission));
    return (uint8_t)(sizeof(hover_mission) / sizeof(hover_mission[0]));
}
