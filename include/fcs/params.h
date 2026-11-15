/**
 * @file params.h
 * @brief Runtime tunable parameter block (gains, limits, safety thresholds).
 *
 * The whole block can be persisted through the HAL. A magic word + version
 * guard against loading a stale/corrupt flash image.
 */
#ifndef FCS_PARAMS_H
#define FCS_PARAMS_H

#include <stdint.h>
#include "fcs/fcs_types.h"
#include "fcs/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FCS_PARAM_MAGIC   0x46435331u   /* "FCS1" */
#define FCS_PARAM_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;

    /* ---- attitude outer loop (rate command = kp * angle error) ---- */
    float att_kp_roll;
    float att_kp_pitch;
    float att_kp_yaw;

    /* ---- rate inner loop (normalised torque = PID of rate error) ---- */
    float rate_kp[3];
    float rate_ki[3];
    float rate_kd[3];
    float rate_ilimit[3];

    /* ---- horizontal position -> desired acceleration ---- */
    float pos_xy_kp;
    float pos_xy_kd;

    /* ---- altitude / vertical speed ---- */
    float alt_kp;
    float vel_z_kp;
    float vel_z_ki;
    float vel_z_ilimit;
    float hover_throttle;

    /* ---- limits ---- */
    float max_tilt_rad;
    float max_yaw_rate_rad;
    float max_rate_rad;
    float max_climb_rate_mps;
    float throttle_min;
    float throttle_max;

    /* ---- estimator (Mahony complementary filter) ---- */
    float est_kp;
    float est_ki;
    float est_kacc;
    float est_kmag;
    float accel_lpf_hz;
    float gyro_bias_limit;

    /* ---- safety ---- */
    float rc_timeout_ms;
    float battery_min_v;
    float battery_warn_v;
    float fault_tilt_rad;
    float loop_timeout_ms;
    uint32_t imu_error_limit;

    /* ---- mission ---- */
    float wp_accept_radius_m;

    /* ---- loop rate ---- */
    float loop_dt_s;          /**< nominal main-loop period, seconds */
} fcs_params_t;

/** Fill with safe defaults for a ~1.2 kg X-quad. */
void fcs_params_defaults(fcs_params_t *p);

/** Clamp every field into a valid range. Returns number of corrected fields. */
int fcs_params_sanitize(fcs_params_t *p);

/** Load from HAL storage; falls back to defaults on any failure. */
void fcs_params_load(fcs_params_t *p, const fcs_hal_t *hal);

/** Persist through HAL storage. Returns false if unsupported or failed. */
bool fcs_params_save(const fcs_params_t *p, const fcs_hal_t *hal);

#ifdef __cplusplus
}
#endif

#endif /* FCS_PARAMS_H */
