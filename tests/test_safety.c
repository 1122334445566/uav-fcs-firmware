#include "test_framework.h"
#include "fcs/failsafe.h"
#include "fcs/state_machine.h"
#include "fcs/telemetry.h"
#include "fcs/params.h"
#include "fcs/fcs_math.h"

#include <string.h>

static fcs_safety_input_t healthy_input(void)
{
    fcs_safety_input_t in;
    memset(&in, 0, sizeof(in));
    in.rc_valid = true;
    in.imu_valid = true;
    in.gnss_valid = true;
    in.armed = true;
    in.battery_v = 16.0f;
    in.tilt_rad = 0.05f;
    in.alt_m = 5.0f;
    in.dt_s = 0.002f;
    in.mode = FCS_MODE_POS_HOLD;
    in.now_us = 1000000u;
    in.last_rc_us = 999000u;
    return in;
}

static int test_failsafe_rules(void)
{
    fcs_params_t p;
    fcs_params_defaults(&p);
    fcs_failsafe_t fs;
    fcs_safety_input_t in;

    fcs_failsafe_init(&fs);
    in = healthy_input();
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_NONE);

    /* RC timeout: emitted once, then suppressed while the fault persists. */
    in.last_rc_us = 0u;
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_RC_LOSS);
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_NONE);
    CHECK(fs.trigger_count == 1u);

    fcs_failsafe_init(&fs);
    in = healthy_input();
    in.battery_v = 12.0f;
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_BATTERY_CRITICAL);

    fcs_failsafe_init(&fs);
    in = healthy_input();
    in.tilt_rad = fcs_deg2rad(70.0f);
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_ATTITUDE_UPSET);
    CHECK_TRUE(fcs_failsafe_is_kill(FCS_FSAFE_ATTITUDE_UPSET));
    CHECK_FALSE(fcs_failsafe_is_kill(FCS_FSAFE_RC_LOSS));

    fcs_failsafe_init(&fs);
    in = healthy_input();
    in.gnss_valid = false;
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_GNSS_LOSS);
    in.mode = FCS_MODE_STABILIZE;
    fcs_failsafe_clear(&fs);
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_NONE);

    fcs_failsafe_init(&fs);
    in = healthy_input();
    in.imu_valid = false;
    fcs_failsafe_reason_t r = FCS_FSAFE_NONE;
    for (uint32_t i = 0; i < p.imu_error_limit + 2u; ++i) {
        in.now_us += 2000u;
        fcs_failsafe_reason_t one = fcs_failsafe_check(&fs, &in, &p);
        if (one != FCS_FSAFE_NONE) r = one;
    }
    CHECK(r == FCS_FSAFE_IMU_ERROR);

    fcs_failsafe_init(&fs);
    in = healthy_input();
    in.dt_s = 0.5f;
    CHECK(fcs_failsafe_check(&fs, &in, &p) == FCS_FSAFE_LOOP_STALL);
    return 0;
}

static int test_state_machine(void)
{
    fcs_params_t p;
    fcs_params_defaults(&p);
    fcs_sm_t sm;
    fcs_sm_init(&sm);
    fcs_state_t s;
    memset(&s, 0, sizeof(s));
    s.battery_v = 16.0f;
    s.quat = fcs_quat_identity();
    s.euler = fcs_quat_to_euler(s.quat);

    fcs_rc_t rc;
    memset(&rc, 0, sizeof(rc));
    rc.valid = true;
    rc.throttle = 0.0f;
    rc.yaw = 1.0f;
    rc.mode_switch = 0u;
    fcs_setpoint_t sp;
    uint64_t now = 1000000u;

    /* High throttle must block pre-arm. */
    rc.throttle = 0.8f;
    CHECK_FALSE(fcs_sm_prearm_check(&sm, &s, &rc, &p));
    rc.throttle = 0.0f;
    CHECK_TRUE(fcs_sm_prearm_check(&sm, &s, &rc, &p));

    /* Hold the arming stick for 1.4 s -> ARMED. */
    for (int i = 0; i < 700; ++i) {
        sm.prearm_ok = fcs_sm_prearm_check(&sm, &s, &rc, &p);
        fcs_sm_update(&sm, &rc, &s, &p, FCS_FSAFE_NONE, now, 0.002f, &sp);
        now += 2000u;
    }
    CHECK(sm.arm_state == FCS_ARM_ARMED);
    CHECK_TRUE(fcs_sm_armed(&sm));

    /* Disarm stick for 0.6 s -> DISARMED. */
    rc.yaw = -1.0f;
    for (int i = 0; i < 300; ++i) {
        sm.prearm_ok = fcs_sm_prearm_check(&sm, &s, &rc, &p);
        fcs_sm_update(&sm, &rc, &s, &p, FCS_FSAFE_NONE, now, 0.002f, &sp);
        now += 2000u;
    }
    CHECK(sm.arm_state == FCS_ARM_DISARMED);

    /* A kill-class fault while armed disarms immediately. */
    rc.yaw = 0.0f;
    fcs_sm_update(&sm, &rc, &s, &p, FCS_FSAFE_NONE, now, 0.002f, &sp);
    sm.arm_state = FCS_ARM_ARMED;
    fcs_sm_update(&sm, &rc, &s, &p, FCS_FSAFE_ATTITUDE_UPSET, now, 0.002f, &sp);
    CHECK(sm.arm_state == FCS_ARM_DISARMED);
    CHECK(sm.active_fault == FCS_FSAFE_ATTITUDE_UPSET);

    /* GNSS loss in POS_HOLD degrades to ALT_HOLD and stays degraded even
     * while the RC switch keeps requesting POS_HOLD. */
    sm.mode = FCS_MODE_POS_HOLD;
    rc.mode_switch = 2u;   /* POS_HOLD */
    fcs_sm_update(&sm, &rc, &s, &p, FCS_FSAFE_GNSS_LOSS, now, 0.002f, &sp);
    CHECK(sm.mode == FCS_MODE_ALT_HOLD);
    CHECK_TRUE(sm.gnss_degraded);
    fcs_sm_update(&sm, &rc, &s, &p, FCS_FSAFE_NONE, now, 0.002f, &sp);
    CHECK(sm.mode == FCS_MODE_ALT_HOLD);
    rc.mode_switch = 0u;

    /* Waypoint sequencing. */
    fcs_sm_t sm2;
    fcs_sm_init(&sm2);
    fcs_waypoint_t wps[2];
    memset(wps, 0, sizeof(wps));
    wps[0].pos_ned = fcs_vec3(0, 0, -5);
    wps[0].alt_m = 5.0f;
    wps[0].acceptance_radius_m = 1.0f;
    wps[0].hold_time_s = 0.1f;
    wps[1].pos_ned = fcs_vec3(10, 0, -5);
    wps[1].alt_m = 5.0f;
    wps[1].acceptance_radius_m = 1.0f;
    wps[1].hold_time_s = 0.1f;
    CHECK_TRUE(fcs_sm_set_waypoints(&sm2, wps, 2));
    s.altitude_m = 5.0f;             /* already at waypoint 0 */
    s.pos_ned = fcs_vec3(0, 0, -5);
    rc.mode_switch = 3u;   /* AUTO */
    for (int i = 0; i < 100; ++i)
        fcs_sm_update(&sm2, &rc, &s, &p, FCS_FSAFE_NONE, now, 0.002f, &sp);
    CHECK(sm2.mode == FCS_MODE_AUTO);
    CHECK(sm2.waypoint_index == 1u);
    CHECK(sm2.mission_active);
    return 0;
}

static int test_params_and_telemetry(void)
{
    fcs_params_t p;
    fcs_params_defaults(&p);
    p.rate_kp[0] = 999.0f;
    p.hover_throttle = -3.0f;
    p.throttle_max = 0.1f;
    p.magic = 0xDEADBEEFu;
    CHECK(fcs_params_sanitize(&p) >= 4);
    CHECK(p.magic == FCS_PARAM_MAGIC);
    CHECK(p.rate_kp[0] <= 3.0f);
    CHECK(p.hover_throttle >= 0.1f);
    CHECK(p.throttle_max > p.throttle_min);

    /* CRC16-CCITT known-answer vector: "123456789" -> 0x29B1. */
    const uint8_t vec[] = "123456789";
    CHECK(fcs_crc16_ccitt(vec, 9, 0xFFFFu) == 0x29B1u);

    uint8_t payload[6] = { 1, 2, 3, 4, 5, 6 };
    uint8_t frame[FCS_TLM_MAX_FRAME];
    uint8_t out[64];
    uint8_t id = 0, seq = 0;
    uint16_t n = fcs_telemetry_encode(frame, sizeof(frame), 42, 7, payload, 6);
    CHECK(n == FCS_TLM_HEADER_SIZE + 6u + FCS_TLM_CRC_SIZE);
    CHECK(frame[0] == FCS_TLM_SYNC);
    int got = fcs_telemetry_decode(frame, n, &id, &seq, out, sizeof(out));
    CHECK(got == 6);
    CHECK(id == 42u && seq == 7u);
    CHECK(memcmp(out, payload, 6) == 0);

    frame[5] ^= 0x55u;   /* corrupt payload */
    CHECK(fcs_telemetry_decode(frame, n, &id, &seq, out, sizeof(out)) == -1);
    frame[5] ^= 0x55u;
    frame[0] = 0x00u;    /* corrupt sync */
    CHECK(fcs_telemetry_decode(frame, n, &id, &seq, out, sizeof(out)) == -1);
    CHECK(fcs_telemetry_encode(frame, 5u, 1, 1, payload, 6) == 0u);

    uint8_t b[16];
    uint8_t o = 0u;
    fcs_tlm_put_f32(b, &o, -12.345f);
    fcs_tlm_put_u32(b, &o, 0xDEADBEEFu);
    o = 0u;
    CHECK_NEAR(fcs_tlm_get_f32(b, &o), -12.345f, 1e-5f);
    CHECK(fcs_tlm_get_u32(b, &o) == 0xDEADBEEFu);
    return 0;
}

int test_safety(void)
{
    if (test_failsafe_rules()) return 1;
    if (test_state_machine()) return 1;
    if (test_params_and_telemetry()) return 1;
    return 0;
}
