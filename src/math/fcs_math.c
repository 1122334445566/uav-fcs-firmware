#include "fcs/fcs_math.h"

#include <math.h>

#define FCS_PI  3.14159265358979323846f
#define FCS_2PI (2.0f * FCS_PI)

float fcs_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float fcs_wrap_pi(float a)
{
    while (a > FCS_PI)  a -= FCS_2PI;
    while (a < -FCS_PI) a += FCS_2PI;
    return a;
}

float fcs_deg2rad(float d) { return d * (FCS_PI / 180.0f); }
float fcs_rad2deg(float r) { return r * (180.0f / FCS_PI); }

float fcs_signf(float v) { return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f); }

float fcs_deadband(float v, float band)
{
    if (band <= 0.0f) return v;
    if (v > band)  return (v - band) / (1.0f - band);
    if (v < -band) return (v + band) / (1.0f - band);
    return 0.0f;
}

fcs_vec3_t fcs_vec3(float x, float y, float z)
{
    fcs_vec3_t v; v.x = x; v.y = y; v.z = z; return v;
}

fcs_vec3_t fcs_vec3_add(fcs_vec3_t a, fcs_vec3_t b)
{ return fcs_vec3(a.x + b.x, a.y + b.y, a.z + b.z); }

fcs_vec3_t fcs_vec3_sub(fcs_vec3_t a, fcs_vec3_t b)
{ return fcs_vec3(a.x - b.x, a.y - b.y, a.z - b.z); }

fcs_vec3_t fcs_vec3_scale(fcs_vec3_t a, float s)
{ return fcs_vec3(a.x * s, a.y * s, a.z * s); }

float fcs_vec3_dot(fcs_vec3_t a, fcs_vec3_t b)
{ return a.x * b.x + a.y * b.y + a.z * b.z; }

fcs_vec3_t fcs_vec3_cross(fcs_vec3_t a, fcs_vec3_t b)
{
    return fcs_vec3(a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x);
}

float fcs_vec3_norm(fcs_vec3_t a)
{ return sqrtf(fcs_vec3_dot(a, a)); }

fcs_vec3_t fcs_vec3_normalize(fcs_vec3_t a)
{
    float n = fcs_vec3_norm(a);
    if (n < 1e-6f) return fcs_vec3(0.0f, 0.0f, 0.0f);
    return fcs_vec3_scale(a, 1.0f / n);
}

fcs_quat_t fcs_quat_identity(void)
{
    fcs_quat_t q; q.q0 = 1.0f; q.q1 = 0.0f; q.q2 = 0.0f; q.q3 = 0.0f; return q;
}

fcs_quat_t fcs_quat_normalize(fcs_quat_t q)
{
    float n = sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
    if (n < 1e-6f) return fcs_quat_identity();
    q.q0 /= n; q.q1 /= n; q.q2 /= n; q.q3 /= n;
    return q;
}

fcs_quat_t fcs_quat_mul(fcs_quat_t a, fcs_quat_t b)
{
    fcs_quat_t q;
    q.q0 = a.q0*b.q0 - a.q1*b.q1 - a.q2*b.q2 - a.q3*b.q3;
    q.q1 = a.q0*b.q1 + a.q1*b.q0 + a.q2*b.q3 - a.q3*b.q2;
    q.q2 = a.q0*b.q2 - a.q1*b.q3 + a.q2*b.q0 + a.q3*b.q1;
    q.q3 = a.q0*b.q3 + a.q1*b.q2 - a.q2*b.q1 + a.q3*b.q0;
    return q;
}

fcs_quat_t fcs_quat_conj(fcs_quat_t q)
{ q.q1 = -q.q1; q.q2 = -q.q2; q.q3 = -q.q3; return q; }

fcs_quat_t fcs_quat_from_euler(fcs_euler_t e)
{
    float cr = cosf(e.roll * 0.5f),  sr = sinf(e.roll * 0.5f);
    float cp = cosf(e.pitch * 0.5f), sp = sinf(e.pitch * 0.5f);
    float cy = cosf(e.yaw * 0.5f),   sy = sinf(e.yaw * 0.5f);
    fcs_quat_t q;
    q.q0 = cr*cp*cy + sr*sp*sy;
    q.q1 = sr*cp*cy - cr*sp*sy;
    q.q2 = cr*sp*cy + sr*cp*sy;
    q.q3 = cr*cp*sy - sr*sp*cy;
    return fcs_quat_normalize(q);
}

fcs_euler_t fcs_quat_to_euler(fcs_quat_t q)
{
    fcs_euler_t e;
    float sinp = 2.0f * (q.q0*q.q2 - q.q3*q.q1);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    e.pitch = asinf(sinp);
    e.roll  = atan2f(2.0f*(q.q0*q.q1 + q.q2*q.q3),
                     1.0f - 2.0f*(q.q1*q.q1 + q.q2*q.q2));
    e.yaw   = atan2f(2.0f*(q.q0*q.q3 + q.q1*q.q2),
                     1.0f - 2.0f*(q.q2*q.q2 + q.q3*q.q3));
    return e;
}

fcs_vec3_t fcs_quat_rotate(fcs_quat_t q, fcs_vec3_t v)
{
    fcs_vec3_t qv = fcs_vec3(q.q1, q.q2, q.q3);
    fcs_vec3_t t  = fcs_vec3_scale(fcs_vec3_cross(qv, v), 2.0f);
    return fcs_vec3_add(fcs_vec3_add(v, fcs_vec3_scale(t, q.q0)),
                        fcs_vec3_cross(qv, t));
}

fcs_vec3_t fcs_quat_rotate_inv(fcs_quat_t q, fcs_vec3_t v)
{ return fcs_quat_rotate(fcs_quat_conj(q), v); }

fcs_quat_t fcs_quat_integrate(fcs_quat_t q, fcs_vec3_t gyro, float dt)
{
    fcs_quat_t omega;
    omega.q0 = 0.0f; omega.q1 = gyro.x; omega.q2 = gyro.y; omega.q3 = gyro.z;
    fcs_quat_t qdot = fcs_quat_mul(q, omega);
    q.q0 += 0.5f * qdot.q0 * dt;
    q.q1 += 0.5f * qdot.q1 * dt;
    q.q2 += 0.5f * qdot.q2 * dt;
    q.q3 += 0.5f * qdot.q3 * dt;
    return fcs_quat_normalize(q);
}

void fcs_lpf_init(fcs_lpf_t *f, float alpha, float initial)
{
    f->alpha = fcs_clampf(alpha, 0.0f, 1.0f);
    f->y = initial;
    f->initialized = true;
}

float fcs_lpf_apply(fcs_lpf_t *f, float x)
{
    if (!f->initialized) {
        f->y = x;
        f->initialized = true;
        return x;
    }
    f->y += f->alpha * (x - f->y);
    return f->y;
}

void fcs_lpf_set_cutoff(fcs_lpf_t *f, float cutoff_hz, float dt)
{
    if (cutoff_hz <= 0.0f || dt <= 0.0f) { f->alpha = 1.0f; return; }
    float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff_hz);
    f->alpha = fcs_clampf(dt / (rc + dt), 0.0f, 1.0f);
}

float fcs_slew(float target, float current, float max_rate, float dt)
{
    if (max_rate <= 0.0f || dt <= 0.0f) return target;
    float max_step = max_rate * dt;
    float d = target - current;
    return current + fcs_clampf(d, -max_step, max_step);
}
