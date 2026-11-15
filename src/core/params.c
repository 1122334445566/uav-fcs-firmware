#include "fcs/params.h"
#include "fcs/fcs_math.h"

#include <string.h>

static void clamp_field(float *v, float lo, float hi, int *fixed)
{
    float old = *v;
    *v = fcs_clampf(old, lo, hi);
    if (*v != old) (*fixed)++;
}

static void clamp_vec3(float *v, float lo, float hi, int *fixed)
{
    for (int i = 0; i < 3; ++i) clamp_field(&v[i], lo, hi, fixed);
}

void fcs_params_defaults(fcs_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->magic    = FCS_PARAM_MAGIC;
    p->version  = FCS_PARAM_VERSION;

    /* Attitude outer loop */
    p->att_kp_roll  = 6.0f;
    p->att_kp_pitch = 6.0f;
    p->att_kp_yaw   = 2.5f;

    /* Rate inner loop */
    p->rate_kp[0] = 0.18f; p->rate_kp[1] = 0.18f; p->rate_kp[2] = 0.22f;
    p->rate_ki[0] = 0.06f; p->rate_ki[1] = 0.06f; p->rate_ki[2] = 0.08f;
    p->rate_kd[0] = 0.004f; p->rate_kd[1] = 0.004f; p->rate_kd[2] = 0.002f;
    p->rate_ilimit[0] = 0.35f; p->rate_ilimit[1] = 0.35f; p->rate_ilimit[2] = 0.35f;

    /* Position */
    p->pos_xy_kp = 1.2f;
    p->pos_xy_kd = 1.8f;

    /* Altitude */
    p->alt_kp         = 1.0f;
    p->vel_z_kp       = 0.16f;
    p->vel_z_ki       = 0.05f;
    p->vel_z_ilimit   = 0.20f;
    p->hover_throttle = 0.50f;

    /* Limits */
    p->max_tilt_rad        = fcs_deg2rad(25.0f);
    p->max_yaw_rate_rad    = 1.5f;
    p->max_rate_rad        = 4.0f;
    p->max_climb_rate_mps  = 2.5f;
    p->throttle_min        = 0.05f;
    p->throttle_max        = 0.95f;

    /* Estimator */
    p->est_kp            = 1.5f;
    p->est_ki            = 0.05f;
    p->est_kacc          = 1.0f;
    p->est_kmag          = 0.0f;   /* magnetometer off until calibrated */
    p->accel_lpf_hz      = 30.0f;
    p->gyro_bias_limit   = 0.05f;

    /* Safety */
    p->rc_timeout_ms     = 500.0f;
    p->battery_min_v     = 13.2f;  /* 4S LiPo under load */
    p->battery_warn_v    = 14.0f;
    p->fault_tilt_rad    = fcs_deg2rad(60.0f);
    p->loop_timeout_ms   = 10.0f;
    p->imu_error_limit   = 20u;
    p->wp_accept_radius_m = 1.5f;
    p->loop_dt_s         = 0.002f; /* 500 Hz */
}

int fcs_params_sanitize(fcs_params_t *p)
{
    int n = 0;
    if (p->magic != FCS_PARAM_MAGIC) { p->magic = FCS_PARAM_MAGIC; n++; }
    if (p->version != FCS_PARAM_VERSION) { p->version = FCS_PARAM_VERSION; n++; }

    clamp_field(&p->att_kp_roll,  0.1f, 30.0f, &n);
    clamp_field(&p->att_kp_pitch, 0.1f, 30.0f, &n);
    clamp_field(&p->att_kp_yaw,   0.1f, 20.0f, &n);
    clamp_vec3(p->rate_kp, 0.0f, 3.0f, &n);
    clamp_vec3(p->rate_ki, 0.0f, 2.0f, &n);
    clamp_vec3(p->rate_kd, 0.0f, 0.2f, &n);
    clamp_vec3(p->rate_ilimit, 0.0f, 1.0f, &n);

    clamp_field(&p->pos_xy_kp, 0.0f, 10.0f, &n);
    clamp_field(&p->pos_xy_kd, 0.0f, 10.0f, &n);
    clamp_field(&p->alt_kp, 0.0f, 5.0f, &n);
    clamp_field(&p->vel_z_kp, 0.0f, 1.0f, &n);
    clamp_field(&p->vel_z_ki, 0.0f, 0.5f, &n);
    clamp_field(&p->vel_z_ilimit, 0.0f, 0.5f, &n);
    clamp_field(&p->hover_throttle, 0.1f, 0.9f, &n);

    clamp_field(&p->max_tilt_rad, 0.05f, 1.0f, &n);
    clamp_field(&p->max_yaw_rate_rad, 0.1f, 6.0f, &n);
    clamp_field(&p->max_rate_rad, 0.5f, 12.0f, &n);
    clamp_field(&p->max_climb_rate_mps, 0.1f, 10.0f, &n);
    clamp_field(&p->throttle_min, 0.0f, 0.3f, &n);
    clamp_field(&p->throttle_max, 0.5f, 1.0f, &n);
    if (p->throttle_max <= p->throttle_min) { p->throttle_max = p->throttle_min + 0.5f; n++; }

    clamp_field(&p->est_kp, 0.0f, 10.0f, &n);
    clamp_field(&p->est_ki, 0.0f, 1.0f, &n);
    clamp_field(&p->est_kacc, 0.0f, 2.0f, &n);
    clamp_field(&p->est_kmag, 0.0f, 2.0f, &n);
    clamp_field(&p->accel_lpf_hz, 1.0f, 200.0f, &n);
    clamp_field(&p->gyro_bias_limit, 0.0f, 0.5f, &n);

    clamp_field(&p->rc_timeout_ms, 50.0f, 10000.0f, &n);
    clamp_field(&p->battery_min_v, 6.0f, 30.0f, &n);
    clamp_field(&p->battery_warn_v, 6.0f, 30.0f, &n);
    if (p->battery_warn_v < p->battery_min_v) { p->battery_warn_v = p->battery_min_v; n++; }
    clamp_field(&p->fault_tilt_rad, 0.3f, 1.5f, &n);
    clamp_field(&p->loop_timeout_ms, 1.0f, 100.0f, &n);
    clamp_field(&p->wp_accept_radius_m, 0.2f, 20.0f, &n);
    clamp_field(&p->loop_dt_s, 0.0005f, 0.02f, &n);
    if (p->imu_error_limit == 0u) { p->imu_error_limit = 20u; n++; }
    return n;
}

void fcs_params_load(fcs_params_t *p, const fcs_hal_t *hal)
{
    bool ok = false;
    if (hal && hal->param_load) {
        fcs_params_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        if (hal->param_load(&tmp, (uint16_t)sizeof(tmp), hal->ctx) &&
            tmp.magic == FCS_PARAM_MAGIC &&
            tmp.version == FCS_PARAM_VERSION) {
            *p = tmp;
            ok = true;
        }
    }
    if (!ok) fcs_params_defaults(p);
    (void)fcs_params_sanitize(p);
}

bool fcs_params_save(const fcs_params_t *p, const fcs_hal_t *hal)
{
    if (!hal || !hal->param_save) return false;
    return hal->param_save(p, (uint16_t)sizeof(*p), hal->ctx);
}
