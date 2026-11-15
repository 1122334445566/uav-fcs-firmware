#include "fcs/attitude.h"

#include <math.h>
#include <string.h>

static const fcs_vec3_t NED_UP = { 0.0f, 0.0f, -1.0f };

void fcs_attitude_init(fcs_attitude_t *est, const fcs_params_t *p)
{
    memset(est, 0, sizeof(*est));
    est->quat = fcs_quat_identity();
    for (int i = 0; i < 3; ++i) {
        fcs_lpf_init(&est->lpf[i], 0.2f, 0.0f);
    }
    est->dt_lpf = p->loop_dt_s;
    for (int i = 0; i < 3; ++i) {
        fcs_lpf_set_cutoff(&est->lpf[i], p->accel_lpf_hz, est->dt_lpf);
    }
    est->initialized = false;
}

void fcs_attitude_reset(fcs_attitude_t *est, const fcs_imu_t *imu)
{
    /* Classic accelerometer tilt initialisation, assumes yaw = current yaw.
     * Specific force points "up" (0,0,-g) when the vehicle is level. */
    float ax = imu->accel.x, ay = imu->accel.y, az = imu->accel.z;
    float roll  = atan2f(-ay, -az);
    float pitch = atan2f(ax, -az);
    float yaw   = fcs_quat_to_euler(est->quat).yaw;

    fcs_euler_t e;
    e.roll = roll; e.pitch = pitch; e.yaw = yaw;
    est->quat = fcs_quat_from_euler(e);

    est->gyro_bias.x = 0.0f; est->gyro_bias.y = 0.0f; est->gyro_bias.z = 0.0f;
    est->error_integral.x = 0.0f;
    est->error_integral.y = 0.0f;
    est->error_integral.z = 0.0f;
    est->accel_lpf = imu->accel;
    for (int i = 0; i < 3; ++i) est->lpf[i].initialized = false;
    est->initialized = true;
    est->update_count = 0u;
}

fcs_quat_t fcs_attitude_update(fcs_attitude_t *est,
                               const fcs_imu_t *imu,
                               const fcs_params_t *p,
                               float dt)
{
    if (dt <= 0.0f) return est->quat;
    if (dt > 0.1f)  dt = 0.1f;

    if (!est->initialized) fcs_attitude_reset(est, imu);

    /* Refresh the LPF coefficient when the loop period changes (e.g. SIL). */
    if (fabsf(dt - est->dt_lpf) > 1e-6f) {
        est->dt_lpf = dt;
        for (int i = 0; i < 3; ++i)
            fcs_lpf_set_cutoff(&est->lpf[i], p->accel_lpf_hz, dt);
    }

    fcs_vec3_t accel;
    accel.x = fcs_lpf_apply(&est->lpf[0], imu->accel.x);
    accel.y = fcs_lpf_apply(&est->lpf[1], imu->accel.y);
    accel.z = fcs_lpf_apply(&est->lpf[2], imu->accel.z);
    est->accel_lpf = accel;

    fcs_vec3_t err = fcs_vec3(0.0f, 0.0f, 0.0f);
    float a_norm = fcs_vec3_norm(accel);

    /* Reject accel reference during free-fall / hard manoeuvres. */
    if (a_norm > 4.0f && a_norm < 20.0f) {
        fcs_vec3_t up_meas = fcs_vec3_scale(accel, 1.0f / a_norm);
        fcs_vec3_t up_est  = fcs_quat_rotate_inv(est->quat, NED_UP);
        err = fcs_vec3_add(err, fcs_vec3_scale(
              fcs_vec3_cross(up_meas, up_est), p->est_kacc));
    }

    if (p->est_kmag > 0.0f && imu->mag_valid) {
        fcs_vec3_t m = fcs_vec3_normalize(imu->mag);
        if (fcs_vec3_norm(m) > 0.5f) {
            /* Mahony: rotate measured field to NED, remove its inclination,
             * rotate the reference back into the body frame, cross-product. */
            fcs_vec3_t h = fcs_quat_rotate(est->quat, m);
            fcs_vec3_t b = fcs_vec3(sqrtf(h.x*h.x + h.y*h.y), 0.0f, h.z);
            b = fcs_vec3_normalize(b);
            fcs_vec3_t w = fcs_quat_rotate_inv(est->quat, b);
            err = fcs_vec3_add(err, fcs_vec3_scale(
                  fcs_vec3_cross(m, w), p->est_kmag));
        }
    }

    /* PI gyro-bias estimator. */
    fcs_vec3_t integ = fcs_vec3_add(est->error_integral,
                                    fcs_vec3_scale(err, p->est_ki * dt));
    float lim = p->gyro_bias_limit;
    integ.x = fcs_clampf(integ.x, -lim, lim);
    integ.y = fcs_clampf(integ.y, -lim, lim);
    integ.z = fcs_clampf(integ.z, -lim, lim);
    est->error_integral = integ;
    est->gyro_bias = integ;

    fcs_vec3_t corr = fcs_vec3_add(fcs_vec3_scale(err, p->est_kp), integ);
    fcs_vec3_t gyro = fcs_vec3_add(imu->gyro, corr);
    est->quat = fcs_quat_integrate(est->quat, gyro, dt);
    est->update_count++;
    return est->quat;
}

void fcs_attitude_fuse_heading(fcs_attitude_t *est, float heading_rad, float gain)
{
    if (!est->initialized || gain <= 0.0f) return;
    float dyaw = fcs_wrap_pi(heading_rad - fcs_quat_to_euler(est->quat).yaw);
    fcs_euler_t e = { 0.0f, 0.0f, fcs_clampf(gain, 0.0f, 1.0f) * dyaw };
    fcs_quat_t dq = fcs_quat_from_euler(e);          /* NED-frame yaw rotation */
    est->quat = fcs_quat_normalize(fcs_quat_mul(dq, est->quat));
}

fcs_euler_t fcs_attitude_euler(const fcs_attitude_t *est)
{ return fcs_quat_to_euler(est->quat); }

fcs_vec3_t fcs_attitude_rates(const fcs_imu_t *imu, const fcs_attitude_t *est)
{ return fcs_vec3_sub(imu->gyro, est->gyro_bias); }
