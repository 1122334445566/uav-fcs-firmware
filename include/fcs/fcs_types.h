/**
 * @file fcs_types.h
 * @brief Core data types for the UAV flight control system.
 *
 * Frames and conventions (IMPORTANT, every module relies on these):
 *   - NED world frame: x = North, y = East, z = Down. Units: metres, m/s.
 *   - Body frame:      x = Forward, y = Right, z = Down (right handed).
 *   - Quaternion q = (q0,q1,q2,q3) = (w,x,y,z), unit, rotates BODY -> NED.
 *   - Euler angles use the ZYX (yaw-pitch-roll) sequence, radians.
 *   - Altitude is positive UP (=-position.z in NED).
 *   - Angular rates are BODY rates (p,q,r) in rad/s: roll, pitch, yaw.
 *   - Normalised actuator / torque commands are in [-1, 1]; thrust in [0, 1].
 */
#ifndef FCS_TYPES_H
#define FCS_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FCS_MAX_MOTORS      8u
#define FCS_MAX_WAYPOINTS   16u
#define FCS_GRAVITY_MPS2    9.80665f

typedef struct {
    float x, y, z;
} fcs_vec3_t;

/** Unit quaternion (w, x, y, z), body -> NED. */
typedef struct {
    float q0, q1, q2, q3;
} fcs_quat_t;

typedef struct {
    float roll, pitch, yaw;
} fcs_euler_t;

typedef struct {
    fcs_vec3_t gyro;      /**< body rates, rad/s */
    fcs_vec3_t accel;     /**< specific force, m/s^2 (points "up" at rest) */
    fcs_vec3_t mag;       /**< body-frame magnetic field, arbitrary units */
    uint64_t   timestamp_us;
    bool       accel_valid;
    bool       mag_valid;
} fcs_imu_t;

typedef struct {
    float altitude_m;     /**< above home, positive up */
    float climb_rate_mps;
    float temperature_c;
    bool  valid;
} fcs_baro_t;

typedef struct {
    double lat_deg, lon_deg;
    float  alt_msl_m;
    float  pos_n_m, pos_e_m;   /**< local NED position relative to home */
    float  vel_n_mps, vel_e_mps, vel_d_mps;
    uint8_t fix_type;          /**< 0 = no fix, 2 = 2D, 3 = 3D */
    uint8_t satellites;
    bool   valid;
} fcs_gnss_t;

typedef struct {
    float roll, pitch, yaw;   /**< sticks, [-1, 1] */
    float throttle;           /**< [0, 1] */
    bool  arm_switch;         /**< true = arm request */
    uint8_t mode_switch;      /**< 0..4, see fcs_flight_mode_t */
    uint64_t last_update_us;
    bool  valid;
} fcs_rc_t;

typedef struct {
    fcs_vec3_t  pos_ned;
    fcs_vec3_t  vel_ned;
    fcs_quat_t  quat;
    fcs_euler_t euler;
    fcs_vec3_t  rates;
    float       altitude_m;
    float       battery_v;
    bool        position_valid;
} fcs_state_t;

typedef enum {
    FCS_MODE_MANUAL = 0,   /**< direct stick -> mixer (rate command only) */
    FCS_MODE_STABILIZE,    /**< attitude hold */
    FCS_MODE_ALT_HOLD,     /**< attitude + barometric altitude hold */
    FCS_MODE_POS_HOLD,     /**< attitude + altitude + GNSS position hold */
    FCS_MODE_AUTO,         /**< waypoint navigation */
    FCS_MODE_COUNT
} fcs_flight_mode_t;

typedef enum {
    FCS_ARM_DISARMED = 0,
    FCS_ARM_ARMING,
    FCS_ARM_ARMED,
    FCS_ARM_LANDING,
    FCS_ARM_FAILSAFE
} fcs_arm_state_t;

typedef struct {
    fcs_flight_mode_t mode;
    fcs_euler_t target_attitude;   /**< rad */
    float       target_yaw_rate;   /**< rad/s */
    float       target_alt_m;      /**< m, positive up */
    fcs_vec3_t  target_pos_ned;    /**< m */
    float       target_throttle;   /**< [0,1], used in MANUAL/STABILIZE */
} fcs_setpoint_t;

typedef struct {
    fcs_vec3_t pos_ned;
    float      alt_m;
    float      yaw_rad;
    float      acceptance_radius_m;
    float      hold_time_s;
} fcs_waypoint_t;

/** Output of the controller, before mixing. */
typedef struct {
    float       thrust;    /**< [0,1] collective */
    fcs_vec3_t  torque;    /**< roll/pitch/yaw, [-1,1] */
} fcs_control_t;

#ifdef __cplusplus
}
#endif

#endif /* FCS_TYPES_H */
