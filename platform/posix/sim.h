/**
 * @file sim.h
 * @brief Software-in-the-loop 6-DOF plant + synthetic sensors.
 *
 * This file is NOT part of the firmware: it exists so the exact same fcs.c
 * binary can be flown on a PC. It also provides a reference BSP showing what
 * a real board support package has to implement.
 */
#ifndef FCS_SIM_H
#define FCS_SIM_H

#include <stdint.h>
#include "fcs/fcs.h"
#include "fcs/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FCS_SIM_HOVER = 0,
    FCS_SIM_WAYPOINT,
    FCS_SIM_FAILSAFE
} fcs_sim_scenario_t;

typedef struct {
    /* ---- plant true state (NED) ---- */
    fcs_vec3_t  pos_ned;
    fcs_vec3_t  vel_ned;
    fcs_vec3_t  accel_ned;      /* includes gravity, for IMU specific force */
    fcs_quat_t  quat;
    fcs_vec3_t  rates;

    /* ---- actuators ---- */
    float       motor_cmd[4];   /* last command written by the flight core */
    float       motors[4];      /* actual output after first-order ESC lag */

    /* ---- environment ---- */
    uint64_t    time_us;
    float       battery_v;
    bool        gnss_available;
    bool        rc_available;
    bool        imu_available;
    fcs_rc_t    rc;

    /* ---- physical constants ---- */
    float mass_kg;
    float ixx, iyy, izz;
    float thrust_max_n;         /* thrust of ONE motor at full output */
    float torque_roll_pitch;    /* N*m per normalised torque command */
    float torque_yaw;
    float motor_tau_s;
    float drag_xy;

    /* ---- instrumentation ---- */
    uint32_t rng_state;
    uint32_t telemetry_bytes;
    uint32_t watchdog_feeds;
    uint32_t motor_writes;
    bool     verbose;
} fcs_sim_t;

void fcs_sim_init(fcs_sim_t *s, uint32_t seed);

/** Advance the plant by dt seconds using the most recent motor commands. */
void fcs_sim_step(fcs_sim_t *s, float dt);

/** Called by the HAL motor_write callback. */
void fcs_sim_write_motors(fcs_sim_t *s, const float *outputs, uint8_t count);

/* Synthetic sensor samplers (called by hal_sim callbacks). */
void fcs_sim_sample_imu(fcs_sim_t *s, fcs_imu_t *out);
void fcs_sim_sample_baro(fcs_sim_t *s, fcs_baro_t *out);
void fcs_sim_sample_gnss(fcs_sim_t *s, fcs_gnss_t *out);

/** Build the HAL table bound to this simulator instance. */
fcs_hal_t fcs_sim_hal(fcs_sim_t *s);

/** Drive the virtual transmitter for a given scenario and mission time. */
void fcs_sim_update_rc(fcs_sim_t *s, fcs_sim_scenario_t scenario, float t_s);

/** Mission definitions used by main_sil.c. */
uint8_t fcs_sim_mission(fcs_sim_scenario_t scenario, fcs_waypoint_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FCS_SIM_H */
