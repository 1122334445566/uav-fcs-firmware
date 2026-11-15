#include "test_framework.h"
#include "fcs/mixer.h"
#include "fcs/fcs_math.h"

#include <string.h>

int test_mixer(void)
{
    fcs_params_t p;
    fcs_params_defaults(&p);
    float m[4];
    bool sat = false;

    /* Pure collective: all four motors identical. */
    fcs_control_t c;
    c.thrust = 0.5f;
    c.torque = fcs_vec3(0, 0, 0);
    fcs_mixer_mix(&c, &p, m, &sat);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(m[i], 0.5f, 1e-6f);
    CHECK_FALSE(sat);

    /* Positive roll -> left motors (2,3) stronger than right (0,1). */
    c.torque = fcs_vec3(0.2f, 0.0f, 0.0f);
    fcs_mixer_mix(&c, &p, m, &sat);
    CHECK(m[2] > m[0]);
    CHECK(m[3] > m[1]);

    /* Positive pitch -> front motors (0,3) stronger than rear (1,2). */
    c.torque = fcs_vec3(0.0f, 0.2f, 0.0f);
    fcs_mixer_mix(&c, &p, m, &sat);
    CHECK(m[0] > m[1]);
    CHECK(m[3] > m[2]);

    /* Positive yaw -> CCW rotors (0,2) stronger than CW (1,3). */
    c.torque = fcs_vec3(0.0f, 0.0f, 0.2f);
    fcs_mixer_mix(&c, &p, m, &sat);
    CHECK(m[0] > m[1]);
    CHECK(m[2] > m[3]);

    /* The plant-side torque reconstruction must invert the mixer exactly. */
    c.torque = fcs_vec3(0.2f, -0.1f, 0.3f);
    c.thrust = 0.5f;
    fcs_mixer_mix(&c, &p, m, &sat);
    fcs_vec3_t t = fcs_mixer_torque_from_outputs(m);
    CHECK_NEAR(t.x, 0.2f, 1e-4f);
    CHECK_NEAR(t.y, -0.1f, 1e-4f);
    CHECK_NEAR(t.z, 0.3f, 1e-4f);

    /* Saturation: outputs stay in range and differential authority survives. */
    c.thrust = 0.95f;
    c.torque = fcs_vec3(0.8f, -0.6f, 0.5f);
    fcs_mixer_mix(&c, &p, m, &sat);
    CHECK_TRUE(sat);
    for (int i = 0; i < 4; ++i) {
        CHECK(m[i] >= p.throttle_min - 1e-6f);
        CHECK(m[i] <= p.throttle_max + 1e-6f);
    }
    CHECK((m[2] - m[0]) > 0.0f);   /* roll sign preserved */

    fcs_mixer_disarm(m, 4);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(m[i], 0.0f, 1e-9f);
    return 0;
}
