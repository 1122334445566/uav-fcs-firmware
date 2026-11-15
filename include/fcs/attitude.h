/**
 * @file attitude.h
 * @brief Mahony complementary attitude estimator (gyro + accel + optional mag).
 *
 * The accelerometer supplies the gravity ("up") reference and the magnetometer
 * supplies the north reference; a PI controller on the gyro bias continuously
 * removes low-frequency drift. No EKF, no dynamic allocation, O(1) per tick:
 * suitable for Cortex-M4F at 1 kHz.
 */
#ifndef FCS_ATTITUDE_H
#define FCS_ATTITUDE_H

#include "fcs/fcs_types.h"
#include "fcs/fcs_math.h"
#include "fcs/params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fcs_quat_t quat;
    fcs_vec3_t gyro_bias;
    fcs_vec3_t error_integral;
    fcs_vec3_t accel_lpf;
    fcs_lpf_t  lpf[3];
    float      dt_lpf;
    bool       initialized;
    uint64_t   last_update_us;
    uint32_t   update_count;
} fcs_attitude_t;

void fcs_attitude_init(fcs_attitude_t *est, const fcs_params_t *p);

/** Reset to level using the current accelerometer vector. */
void fcs_attitude_reset(fcs_attitude_t *est, const fcs_imu_t *imu);

/**
 * Integrate one IMU sample.
 * @param dt seconds, must be in (0, 0.1]
 * @return the updated unit quaternion (body -> NED)
 */
fcs_quat_t fcs_attitude_update(fcs_attitude_t *est,
                               const fcs_imu_t *imu,
                               const fcs_params_t *p,
                               float dt);

/** Fuse an external heading observation (GNSS course / vision), gain in [0,1). */
void fcs_attitude_fuse_heading(fcs_attitude_t *est, float heading_rad, float gain);

fcs_euler_t fcs_attitude_euler(const fcs_attitude_t *est);
fcs_vec3_t  fcs_attitude_rates(const fcs_imu_t *imu, const fcs_attitude_t *est);

#ifdef __cplusplus
}
#endif

#endif /* FCS_ATTITUDE_H */
