/**
 * @file main_sil.c
 * @brief Closed-loop software-in-the-loop harness.
 *
 *   ./sil --scenario hover|waypoint|failsafe --seconds 20 --out trace.csv
 *
 * The flight core is byte-for-byte the same code that runs on an MCU; only
 * the HAL table differs. The harness asserts tracking accuracy and safety
 * behaviour, which makes it usable as a CI regression test.
 */
#include "sim.h"
#include "fcs/fcs_math.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIL_DT 0.002f   /* 500 Hz */

typedef struct {
    float max_alt_err;
    float max_pos_err;
    float max_tilt_rad;
    float max_alt;
    uint32_t fault_events;
    uint32_t nan_events;
    uint32_t csv_rows;
    bool armed_ever;
    bool landing_seen;
    fcs_arm_state_t final_arm;
    uint8_t final_wp;
    bool mission_complete;
} sil_stats_t;

static const char *scenario_name(fcs_sim_scenario_t s)
{
    return s == FCS_SIM_WAYPOINT ? "waypoint" :
           s == FCS_SIM_FAILSAFE ? "failsafe" : "hover";
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--scenario hover|waypoint|failsafe] [--seconds N]\n"
        "          [--out FILE] [--seed N] [--verbose] [--no-compass]\n"
        "  --no-compass  run without magnetic heading (gyro-only yaw)\n", prog);
}

int main(int argc, char **argv)
{
    fcs_sim_scenario_t scenario = FCS_SIM_HOVER;
    float seconds = 0.0f;
    const char *out_path = "sil_trace.csv";
    uint32_t seed = 0x1234ABCDu;
    bool verbose = false;
    bool compass = true;   /* the simulator provides a calibrated magnetometer */

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--scenario") && i + 1 < argc) {
            const char *v = argv[++i];
            scenario = !strcmp(v, "waypoint") ? FCS_SIM_WAYPOINT :
                       !strcmp(v, "failsafe") ? FCS_SIM_FAILSAFE : FCS_SIM_HOVER;
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "--out") && i + 1 < argc) {
            out_path = argv[++i];
        } else if (!strcmp(argv[i], "--seed") && i + 1 < argc) {
            seed = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--verbose")) {
            verbose = true;
        } else if (!strcmp(argv[i], "--no-compass")) {
            compass = false;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (seconds <= 0.0f) {
        seconds = scenario == FCS_SIM_WAYPOINT ? 55.0f :
                  scenario == FCS_SIM_FAILSAFE ? 30.0f : 22.0f;
    }

    fcs_sim_t sim;
    fcs_sim_init(&sim, seed);
    sim.verbose = verbose;
    fcs_hal_t hal = fcs_sim_hal(&sim);

    fcs_params_t params;
    fcs_params_defaults(&params);
    /* Production firmware keeps the compass OFF until it is calibrated.
     * The simulator supplies an ideal, perfectly calibrated field, so it
     * exercises the magnetometer fusion path by default. */
    params.est_kmag = compass ? 0.5f : 0.0f;

    fcs_t fcs;
    if (!fcs_init(&fcs, &hal, &params)) {
        fprintf(stderr, "fcs_init failed (invalid HAL)\n");
        return 2;
    }
    fcs_waypoint_t mission[FCS_MAX_WAYPOINTS];
    uint8_t wp_count = fcs_sim_mission(scenario, mission);
    if (!fcs_set_waypoints(&fcs, mission, wp_count)) {
        fprintf(stderr, "failed to install mission\n");
        return 2;
    }

    FILE *csv = fopen(out_path, "w");
    if (!csv) {
        fprintf(stderr, "cannot open %s for writing\n", out_path);
        return 2;
    }
    fprintf(csv, "t_s,alt_m,alt_sp_m,pos_n_m,pos_e_m,pos_n_sp_m,pos_e_sp_m,"
                "roll_rad,pitch_rad,yaw_rad,vn_mps,ve_mps,vz_mps,"
                "m0,m1,m2,m3,thrust,arm,mode,fault\n");

    sil_stats_t st;
    memset(&st, 0, sizeof(st));
    st.final_arm = FCS_ARM_DISARMED;

    const uint64_t steps = (uint64_t)(seconds / SIL_DT);
    for (uint64_t i = 0; i < steps; ++i) {
        float t = (float)i * SIL_DT;
        sim.time_us = (uint64_t)((double)i * SIL_DT * 1000000.0 + 0.5);
        fcs_sim_update_rc(&sim, scenario, t);

        fcs_step(&fcs, sim.time_us);
        fcs_sim_step(&sim, SIL_DT);

        const fcs_state_t *state = fcs_state(&fcs);
        const fcs_sm_t *sm = fcs_state_machine(&fcs);
        float alt = state->altitude_m;
        float tilt = sqrtf(state->euler.roll * state->euler.roll +
                           state->euler.pitch * state->euler.pitch);

        if (!isfinite(alt) || !isfinite(state->pos_ned.x) ||
            !isfinite(state->euler.roll)) st.nan_events++;
        if (alt > st.max_alt) st.max_alt = alt;
        if (tilt > st.max_tilt_rad) st.max_tilt_rad = tilt;
        if (sm->arm_state == FCS_ARM_ARMED) st.armed_ever = true;
        if (sm->arm_state == FCS_ARM_LANDING) st.landing_seen = true;
        st.fault_events = fcs.failsafe.trigger_count;
        st.final_arm = sm->arm_state;
        st.final_wp = sm->waypoint_index;
        st.mission_complete = sm->mission_complete;

        /* Steady-state tracking window, excluding take-off and any emergency. */
        bool steady = (t > 8.0f && t < seconds - 0.5f) &&
                      (sm->arm_state == FCS_ARM_ARMED) &&
                      (scenario != FCS_SIM_FAILSAFE || t < 11.0f);
        if (steady) {
            float ae = fabsf(alt - fcs.setpoint.target_alt_m);
            float pe = sqrtf(
                (state->pos_ned.x - fcs.setpoint.target_pos_ned.x) *
                (state->pos_ned.x - fcs.setpoint.target_pos_ned.x) +
                (state->pos_ned.y - fcs.setpoint.target_pos_ned.y) *
                (state->pos_ned.y - fcs.setpoint.target_pos_ned.y));
            if (ae > st.max_alt_err) st.max_alt_err = ae;
            if (pe > st.max_pos_err) st.max_pos_err = pe;
        }

        if (i % 10u == 0u) {
            fprintf(csv,
                "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d\n",
                t, alt, fcs.setpoint.target_alt_m,
                state->pos_ned.x, state->pos_ned.y,
                fcs.setpoint.target_pos_ned.x, fcs.setpoint.target_pos_ned.y,
                state->euler.roll, state->euler.pitch, state->euler.yaw,
                state->vel_ned.x, state->vel_ned.y, state->vel_ned.z,
                fcs.motors[0], fcs.motors[1], fcs.motors[2], fcs.motors[3],
                fcs.control.thrust, (int)sm->arm_state, (int)sm->mode,
                (int)fcs.failsafe.reason);
            st.csv_rows++;
        }
    }
    fclose(csv);

    printf("=== SIL [%s] %.1f s @ %.0f Hz ===\n",
           scenario_name(scenario), seconds, 1.0f / SIL_DT);
    printf("loops            : %u\n", fcs.loop_count);
    printf("final arm state  : %s\n", fcs_sm_arm_name(st.final_arm));
    printf("final mode       : %s\n", fcs_sm_mode_name(fcs.sm.mode));
    printf("final alt / pos  : %.3f m / (%.2f, %.2f) m\n",
           -sim.pos_ned.z, sim.pos_ned.x, sim.pos_ned.y);
    printf("max altitude     : %.3f m\n", st.max_alt);
    printf("max tilt         : %.2f deg\n", fcs_rad2deg(st.max_tilt_rad));
    printf("steady alt error : %.3f m (max)\n", st.max_alt_err);
    printf("steady pos error : %.3f m (max)\n", st.max_pos_err);
    printf("waypoint index   : %u / %u (complete=%s)\n",
           (unsigned)st.final_wp + 1u, (unsigned)wp_count,
           st.mission_complete ? "yes" : "no");
    printf("failsafe events  : %u\n", st.fault_events);
    printf("landing seen     : %s\n", st.landing_seen ? "yes" : "no");
    printf("telemetry bytes  : %u\n", sim.telemetry_bytes);
    printf("watchdog feeds   : %u\n", sim.watchdog_feeds);
    printf("CSV              : %s (%u rows)\n", out_path, st.csv_rows);

    bool pass = st.armed_ever && st.nan_events == 0u && st.max_alt < 30.0f
             && st.max_tilt_rad < fcs_deg2rad(50.0f);
    if (scenario == FCS_SIM_HOVER) {
        pass = pass && st.max_alt_err < 0.6f && st.max_pos_err < 1.0f
             && st.fault_events == 0u && st.final_arm == FCS_ARM_ARMED;
    } else if (scenario == FCS_SIM_WAYPOINT) {
        pass = pass && st.mission_complete && st.final_arm == FCS_ARM_DISARMED;
    } else {
        pass = pass && st.fault_events >= 1u && st.landing_seen
             && st.final_arm == FCS_ARM_DISARMED;
    }
    printf("RESULT           : %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
