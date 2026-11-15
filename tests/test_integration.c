/**
 * @file test_integration.c
 * @brief Exercises the top-level scheduler with a mock (non-simulator) HAL:
 *        arming interlock, motor kill, watchdog, telemetry cadence.
 */
#include "test_framework.h"
#include "fcs/fcs.h"
#include "fcs/fcs_math.h"

#include <string.h>

typedef struct {
    uint64_t now_us;
    fcs_rc_t rc;
    float    battery_v;
    bool     imu_available;
    float    motors[4];
    uint32_t motor_writes;
    uint32_t tlm_frames;
    uint32_t watchdog_feeds;
    uint32_t log_lines;
} mock_t;

static bool m_imu(fcs_imu_t *out, void *ctx)
{
    mock_t *m = (mock_t *)ctx;
    if (!m->imu_available) return false;
    out->gyro = fcs_vec3(0, 0, 0);
    out->accel = fcs_vec3(0, 0, -FCS_GRAVITY_MPS2);
    out->mag = fcs_vec3(0.35f, 0.0f, 0.60f);
    out->timestamp_us = m->now_us;
    out->accel_valid = true;
    out->mag_valid = true;
    return true;
}

static bool m_baro(fcs_baro_t *out, void *ctx)
{
    (void)ctx;
    out->altitude_m = 0.0f;
    out->climb_rate_mps = 0.0f;
    out->temperature_c = 25.0f;
    out->valid = true;
    return true;
}

static bool m_gnss(fcs_gnss_t *out, void *ctx)
{
    (void)ctx;
    memset(out, 0, sizeof(*out));
    out->fix_type = 3u;
    out->satellites = 12u;
    out->valid = true;
    return true;
}

static bool m_rc(fcs_rc_t *out, void *ctx)
{
    mock_t *m = (mock_t *)ctx;
    *out = m->rc;
    out->last_update_us = m->now_us;
    return true;
}

static bool m_battery(float *v, void *ctx)
{
    *v = ((mock_t *)ctx)->battery_v;
    return true;
}

static void m_motor(const float *outputs, uint8_t count, void *ctx)
{
    mock_t *m = (mock_t *)ctx;
    if (count > 4u) count = 4u;
    for (uint8_t i = 0; i < count; ++i) m->motors[i] = outputs[i];
    m->motor_writes++;
}

static uint64_t m_time(void *ctx) { return ((mock_t *)ctx)->now_us; }
static void m_log(fcs_log_level_t level, const char *msg, void *ctx)
{
    (void)level; (void)msg;
    ((mock_t *)ctx)->log_lines++;
}
static void m_tlm(const uint8_t *bytes, uint16_t len, void *ctx)
{
    (void)bytes; (void)len;
    ((mock_t *)ctx)->tlm_frames++;
}
static void m_wd(void *ctx) { ((mock_t *)ctx)->watchdog_feeds++; }

static fcs_hal_t mock_hal(mock_t *m)
{
    fcs_hal_t h;
    memset(&h, 0, sizeof(h));
    h.imu_read = m_imu;   h.baro_read = m_baro;  h.gnss_read = m_gnss;
    h.rc_read = m_rc;     h.battery_read = m_battery;
    h.motor_write = m_motor; h.time_us = m_time; h.log = m_log;
    h.telemetry_write = m_tlm; h.watchdog_feed = m_wd;
    h.ctx = m;
    return h;
}

static void run_ticks(fcs_t *f, mock_t *m, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i) {
        m->now_us += 2000u;
        fcs_step(f, m->now_us);
    }
}

int test_integration(void)
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.battery_v = 16.0f;
    m.imu_available = true;
    m.rc.valid = true;
    m.rc.throttle = 0.0f;
    m.rc.yaw = 1.0f;
    m.rc.mode_switch = 1u;   /* ALT_HOLD */
    fcs_hal_t hal = mock_hal(&m);

    fcs_t f;
    CHECK_TRUE(fcs_hal_validate(&hal));
    CHECK_TRUE(fcs_init(&f, &hal, NULL));
    CHECK(f.ready);
    CHECK(f.params.magic == FCS_PARAM_MAGIC);

    /* Before arming, no matter what the sticks say, motors must stay at zero. */
    float dt = fcs_step(&f, (m.now_us += 2000u));
    CHECK_NEAR(dt, 0.002f, 1e-4f);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(m.motors[i], 0.0f, 1e-9f);

    /* Stick arming sequence. */
    run_ticks(&f, &m, 700u);
    CHECK(f.sm.arm_state == FCS_ARM_ARMED);
    CHECK(m.motors[0] > 0.0f);
    CHECK(m.watchdog_feeds > 700u);
    CHECK(m.tlm_frames > 0u);       /* 4 frames every 50 loops */
    CHECK(m.log_lines > 0u);

    /* Disarm stick kills the motors. */
    m.rc.yaw = -1.0f;
    run_ticks(&f, &m, 300u);
    CHECK(f.sm.arm_state == FCS_ARM_DISARMED);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(m.motors[i], 0.0f, 1e-9f);
    CHECK_NEAR(f.control.thrust, 0.0f, 1e-9f);

    /* Arm again, then lose the IMU: the failsafe must kill the motors. */
    m.rc.yaw = 1.0f;
    run_ticks(&f, &m, 700u);
    CHECK(f.sm.arm_state == FCS_ARM_ARMED);
    m.rc.yaw = 0.0f;
    m.rc.throttle = 0.5f;
    m.imu_available = false;
    uint32_t faults_before = f.failsafe.trigger_count;
    run_ticks(&f, &m, 40u);
    CHECK(f.failsafe.trigger_count > faults_before);
    CHECK(f.sm.arm_state == FCS_ARM_DISARMED);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(m.motors[i], 0.0f, 1e-9f);

    /* A mode request and a waypoint mission go through the public API. */
    fcs_request_mode(&f, FCS_MODE_POS_HOLD);
    CHECK(f.sm.mode == FCS_MODE_POS_HOLD);
    fcs_waypoint_t wp;
    memset(&wp, 0, sizeof(wp));
    wp.pos_ned = fcs_vec3(1, 2, -3);
    wp.alt_m = 3.0f;
    CHECK_TRUE(fcs_set_waypoints(&f, &wp, 1u));
    CHECK(f.sm.waypoint_count == 1u);
    CHECK(fcs_state(&f) == &f.state);
    CHECK(fcs_state_machine(&f) == &f.sm);
    return 0;
}
