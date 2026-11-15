#include "test_framework.h"
#include "fcs/attitude.h"
#include "fcs/fcs_math.h"

static fcs_imu_t level_imu(void)
{
    fcs_imu_t imu;
    imu.gyro = fcs_vec3(0.0f, 0.0f, 0.0f);
    imu.accel = fcs_vec3(0.0f, 0.0f, -FCS_GRAVITY_MPS2);
    imu.mag = fcs_vec3(0.35f, 0.0f, 0.60f);
    imu.timestamp_us = 0u;
    imu.accel_valid = true;
    imu.mag_valid = true;
    return imu;
}

int test_attitude(void)
{
    fcs_params_t p;
    fcs_params_defaults(&p);
    fcs_attitude_t est;
    fcs_attitude_init(&est, &p);
    fcs_imu_t imu = level_imu();

    /* Level, no rotation: attitude stays level. */
    for (int i = 0; i < 500; ++i)
        fcs_attitude_update(&est, &imu, &p, 0.002f);
    fcs_euler_t e = fcs_attitude_euler(&est);
    CHECK_NEAR(e.roll, 0.0f, 0.01f);
    CHECK_NEAR(e.pitch, 0.0f, 0.01f);

    /* Now present a 20 deg roll / -15 deg pitch gravity vector; the
     * complementary filter must converge from the level estimate. */
    fcs_euler_t truth;
    truth.roll = fcs_deg2rad(20.0f);
    truth.pitch = fcs_deg2rad(-15.0f);
    truth.yaw = 0.4f;
    fcs_quat_t qt = fcs_quat_from_euler(truth);
    imu.accel = fcs_quat_rotate_inv(qt,
                    fcs_vec3(0.0f, 0.0f, -FCS_GRAVITY_MPS2));
    for (int i = 0; i < 1500; ++i)
        fcs_attitude_update(&est, &imu, &p, 0.002f);
    e = fcs_attitude_euler(&est);
    CHECK_NEAR(e.roll, truth.roll, 0.03f);
    CHECK_NEAR(e.pitch, truth.pitch, 0.03f);

    /* Gyro bias estimation must grow toward a real constant bias. */
    fcs_attitude_t est2;
    fcs_attitude_init(&est2, &p);
    imu = level_imu();
    imu.gyro = fcs_vec3(0.02f, 0.0f, 0.0f);
    for (int i = 0; i < 2000; ++i)
        fcs_attitude_update(&est2, &imu, &p, 0.002f);
    CHECK(est2.gyro_bias.x < -0.001f);   /* correction opposes the bias */
    CHECK(fabsf(est2.gyro_bias.x) <= p.gyro_bias_limit + 1e-6f);

    /* External heading fusion moves yaw toward the observation. */
    fcs_attitude_t est3;
    fcs_attitude_init(&est3, &p);
    imu = level_imu();
    fcs_attitude_update(&est3, &imu, &p, 0.002f);
    float yaw0 = fcs_attitude_euler(&est3).yaw;
    for (int i = 0; i < 100; ++i)
        fcs_attitude_fuse_heading(&est3, 1.0f, 0.1f);
    float yaw1 = fcs_attitude_euler(&est3).yaw;
    CHECK(yaw1 > yaw0 + 0.5f);
    CHECK(yaw1 <= 1.001f);

    /* Quaternion must stay normalised. */
    float n2 = est.quat.q0*est.quat.q0 + est.quat.q1*est.quat.q1
             + est.quat.q2*est.quat.q2 + est.quat.q3*est.quat.q3;
    CHECK_NEAR(n2, 1.0f, 1e-4f);
    return 0;
}
