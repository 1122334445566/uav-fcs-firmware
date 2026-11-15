/**
 * @file fcs_math.h
 * @brief Dependency-free float math helpers (C99, only libm required).
 */
#ifndef FCS_MATH_H
#define FCS_MATH_H

#include "fcs/fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

float fcs_clampf(float v, float lo, float hi);
float fcs_wrap_pi(float angle_rad);
float fcs_deg2rad(float deg);
float fcs_rad2deg(float rad);
float fcs_signf(float v);
float fcs_deadband(float v, float band);

fcs_vec3_t fcs_vec3(float x, float y, float z);
fcs_vec3_t fcs_vec3_add(fcs_vec3_t a, fcs_vec3_t b);
fcs_vec3_t fcs_vec3_sub(fcs_vec3_t a, fcs_vec3_t b);
fcs_vec3_t fcs_vec3_scale(fcs_vec3_t a, float s);
float      fcs_vec3_dot(fcs_vec3_t a, fcs_vec3_t b);
fcs_vec3_t fcs_vec3_cross(fcs_vec3_t a, fcs_vec3_t b);
float      fcs_vec3_norm(fcs_vec3_t a);
fcs_vec3_t fcs_vec3_normalize(fcs_vec3_t a);

fcs_quat_t fcs_quat_identity(void);
fcs_quat_t fcs_quat_normalize(fcs_quat_t q);
fcs_quat_t fcs_quat_mul(fcs_quat_t a, fcs_quat_t b);      /**< Hamilton product a (x) b */
fcs_quat_t fcs_quat_conj(fcs_quat_t q);
fcs_quat_t fcs_quat_from_euler(fcs_euler_t e);
fcs_euler_t fcs_quat_to_euler(fcs_quat_t q);
/** Rotate a vector from body to NED (or NED to body with fcs_quat_rotate_inv). */
fcs_vec3_t fcs_quat_rotate(fcs_quat_t q, fcs_vec3_t v);
fcs_vec3_t fcs_quat_rotate_inv(fcs_quat_t q, fcs_vec3_t v);
/** First order quaternion integration with body rates over dt seconds. */
fcs_quat_t fcs_quat_integrate(fcs_quat_t q, fcs_vec3_t gyro_rad_s, float dt);

/** Simple first-order low pass filter: y += alpha * (x - y). */
typedef struct {
    float alpha;
    float y;
    bool  initialized;
} fcs_lpf_t;

void  fcs_lpf_init(fcs_lpf_t *f, float alpha, float initial);
float fcs_lpf_apply(fcs_lpf_t *f, float x);
void  fcs_lpf_set_cutoff(fcs_lpf_t *f, float cutoff_hz, float dt);

/** Slew-rate limiter, returns the value clamped so |dv| <= max_rate*dt. */
float fcs_slew(float target, float current, float max_rate, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FCS_MATH_H */
