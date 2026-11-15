#include "test_framework.h"
#include "fcs/controller.h"
#include "fcs/fcs_math.h"

#include <string.h>

static fcs_state_t hovering_state(void)
{
    fcs_state_t s;
    memset(&s, 0, sizeof(s));
    s.quat = fcs_quat_identity();
    s.euler = fcs_quat_to_euler(s.quat);
    s.rates = fcs_vec3(0, 0, 0);
    s.pos_ned = fcs_vec3(0, 0, -5.0f);
    s.vel_ned = fcs_vec3(0, 0, 0);
    s.altitude_m = 5.0f;
    s.battery_v = 16.0f;
    s.position_valid = true;
    return s;
}

int test_control(void)
{
    fcs_params_t p;
    fcs_params_defaults(&p);
    fcs_controller_t c;
    fcs_controller_init(&c);
    fcs_state_t s = hovering_state();
    fcs_setpoint_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.target_throttle = 0.5f;
    sp.target_alt_m = 5.0f;
    sp.target_pos_ned = fcs_vec3(0, 0, -5.0f);

    /* Trimmed hover produces zero torque and the commanded throttle. */
    fcs_control_t out = fcs_controller_update(&c, &s, &sp, FCS_MODE_STABILIZE, &p, 0.002f);
    CHECK_NEAR(out.thrust, 0.5f, 1e-5f);
    CHECK_NEAR(out.torque.x, 0.0f, 1e-5f);
    CHECK_NEAR(out.torque.y, 0.0f, 1e-5f);
    CHECK_NEAR(out.torque.z, 0.0f, 1e-5f);

    /* A positive roll error must produce a positive roll torque. */
    sp.target_attitude.roll = 0.10f;
    out = fcs_controller_update(&c, &s, &sp, FCS_MODE_STABILIZE, &p, 0.002f);
    CHECK(out.torque.x > 0.0f);
    CHECK_NEAR(out.torque.y, 0.0f, 1e-5f);

    /* Altitude hold: a 3 m altitude error must increase collective thrust. */
    fcs_controller_reset(&c);
    sp.target_attitude.roll = 0.0f;
    s.altitude_m = 2.0f;
    sp.target_alt_m = 5.0f;
    out = fcs_controller_update(&c, &s, &sp, FCS_MODE_ALT_HOLD, &p, 0.002f);
    CHECK(out.thrust > p.hover_throttle);
    CHECK(out.thrust <= p.throttle_max);

    /* Position hold: a target 2 m North with zero yaw needs negative pitch. */
    fcs_controller_reset(&c);
    s = hovering_state();
    sp.target_pos_ned = fcs_vec3(2.0f, 0.0f, -5.0f);
    sp.target_attitude.yaw = 0.0f;
    out = fcs_controller_update(&c, &s, &sp, FCS_MODE_POS_HOLD, &p, 0.002f);
    CHECK(c.desired_attitude.pitch < 0.0f);   /* nose-down (negative pitch) -> North */
    CHECK(fabsf(c.desired_attitude.pitch) <= p.max_tilt_rad + 1e-5f);

    /* Integral anti-windup under a persistent error. */
    fcs_controller_reset(&c);
    s = hovering_state();
    s.euler.roll = -0.5f;
    for (int i = 0; i < 5000; ++i)
        fcs_controller_update(&c, &s, &sp, FCS_MODE_POS_HOLD, &p, 0.002f);
    CHECK(fabsf(c.rate_integral.x) <= p.rate_ilimit[0] + 1e-6f);
    CHECK(fabsf(c.rate_integral.y) <= p.rate_ilimit[1] + 1e-6f);

    /* Tilt conversion is the inverse of the small-angle thrust model:
     * yaw = 90 deg, desire acceleration North -> pure negative roll. */
    fcs_euler_t t = fcs_accel_to_tilt(fcs_vec3(1.0f, 0.0f, 0.0f),
                                      1.5707963f, p.max_tilt_rad);
    CHECK_NEAR(t.pitch, 0.0f, 1e-3f);
    CHECK(t.roll < 0.0f);
    fcs_euler_t t2 = fcs_accel_to_tilt(fcs_vec3(1.0f, 0.0f, 0.0f),
                                       0.0f, p.max_tilt_rad);
    CHECK(t2.pitch < 0.0f);
    CHECK_NEAR(t2.roll, 0.0f, 1e-3f);
    return 0;
}
