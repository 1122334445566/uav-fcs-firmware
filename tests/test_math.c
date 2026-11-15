#include "test_framework.h"
#include "fcs/fcs_math.h"

int test_math(void)
{
    CHECK_NEAR(fcs_clampf(2.0f, -1.0f, 1.0f), 1.0f, 1e-6f);
    CHECK_NEAR(fcs_clampf(-2.0f, -1.0f, 1.0f), -1.0f, 1e-6f);
    CHECK_NEAR(fcs_wrap_pi(3.0f * 3.14159265f), 3.14159265f, 1e-4f);
    CHECK_NEAR(fcs_deg2rad(180.0f), 3.14159265f, 1e-5f);
    CHECK_NEAR(fcs_deadband(0.02f, 0.10f), 0.0f, 1e-6f);
    CHECK(fcs_deadband(0.8f, 0.10f) > 0.7f);

    fcs_vec3_t a = fcs_vec3(1, 0, 0), b = fcs_vec3(0, 1, 0);
    CHECK_NEAR(fcs_vec3_dot(a, b), 0.0f, 1e-6f);
    CHECK_NEAR(fcs_vec3_cross(a, b).z, 1.0f, 1e-6f);
    CHECK_NEAR(fcs_vec3_norm(fcs_vec3(3, 4, 0)), 5.0f, 1e-5f);
    CHECK_NEAR(fcs_vec3_norm(fcs_vec3_normalize(fcs_vec3(0, 0, 7))), 1.0f, 1e-5f);

    /* Euler -> quaternion -> Euler round trip for several attitudes. */
    for (int i = 0; i < 5; ++i) {
        fcs_euler_t e = { -0.4f + 0.2f * i, -0.3f + 0.15f * i, -1.0f + 0.5f * i };
        fcs_quat_t q = fcs_quat_from_euler(e);
        CHECK_NEAR(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3, 1.0f, 1e-5f);
        fcs_euler_t r = fcs_quat_to_euler(q);
        CHECK_NEAR(r.roll,  e.roll,  1e-4f);
        CHECK_NEAR(r.pitch, e.pitch, 1e-4f);
        CHECK_NEAR(r.yaw,   e.yaw,   1e-4f);
    }

    /* Rotate a vector and rotate it back. */
    fcs_euler_t e0;
    e0.roll = 0.3f; e0.pitch = -0.2f; e0.yaw = 0.8f;
    fcs_quat_t q = fcs_quat_from_euler(e0);
    fcs_vec3_t v = fcs_vec3(1.2f, -0.4f, 0.7f);
    fcs_vec3_t back = fcs_quat_rotate_inv(q, fcs_quat_rotate(q, v));
    CHECK_NEAR(back.x, v.x, 1e-4f);
    CHECK_NEAR(back.y, v.y, 1e-4f);
    CHECK_NEAR(back.z, v.z, 1e-4f);

    /* Integrate a constant yaw rate of 1 rad/s for 1 s (1000 x 1 ms). */
    fcs_quat_t qi = fcs_quat_identity();
    fcs_vec3_t w = fcs_vec3(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 1000; ++i) qi = fcs_quat_integrate(qi, w, 0.001f);
    CHECK_NEAR(fcs_quat_to_euler(qi).yaw, 1.0f, 0.01f);

    /* Low-pass filter converges; slew limiter respects the rate bound. */
    fcs_lpf_t lpf;
    fcs_lpf_init(&lpf, 0.1f, 0.0f);
    for (int i = 0; i < 200; ++i) fcs_lpf_apply(&lpf, 1.0f);
    CHECK_NEAR(lpf.y, 1.0f, 1e-3f);
    CHECK_NEAR(fcs_slew(1.0f, 0.0f, 2.0f, 0.1f), 0.2f, 1e-6f);
    return 0;
}
