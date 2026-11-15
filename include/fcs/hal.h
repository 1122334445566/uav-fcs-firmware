/**
 * @file hal.h
 * @brief Hardware abstraction layer.
 *
 * The flight core NEVER calls a vendor SDK directly. A board support package
 * fills in this table; the POSIX SIL simulation fills in another. Every
 * callback receives the same user context pointer (hal->ctx).
 *
 * Notes for MCU ports:
 *   - imu_read/baro_read/gnss_read are non-blocking: return false when no new
 *     sample is available; the core keeps the previous state.
 *   - motor_write must accept values in [0,1] and convert to DShot/PWM there.
 *   - time_us should be monotonic (e.g. a 32-bit timer extended to 64-bit).
 */
#ifndef FCS_HAL_H
#define FCS_HAL_H

#include <stdint.h>
#include "fcs/fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FCS_LOG_DEBUG = 0,
    FCS_LOG_INFO,
    FCS_LOG_WARN,
    FCS_LOG_ERROR
} fcs_log_level_t;

typedef struct fcs_hal {
    bool (*imu_read)(fcs_imu_t *out, void *ctx);
    bool (*baro_read)(fcs_baro_t *out, void *ctx);
    bool (*gnss_read)(fcs_gnss_t *out, void *ctx);
    bool (*rc_read)(fcs_rc_t *out, void *ctx);
    bool (*battery_read)(float *voltage_v, void *ctx);

    /** outputs[i] in [0,1] for i in [0,count). */
    void (*motor_write)(const float *outputs, uint8_t count, void *ctx);

    uint64_t (*time_us)(void *ctx);
    void (*log)(fcs_log_level_t level, const char *message, void *ctx);
    void (*telemetry_write)(const uint8_t *bytes, uint16_t len, void *ctx);

    /** Persistent parameter storage (flash / EEPROM / file). Optional. */
    bool (*param_load)(void *buffer, uint16_t size, void *ctx);
    bool (*param_save)(const void *buffer, uint16_t size, void *ctx);

    void (*watchdog_feed)(void *ctx);

    void *ctx;
} fcs_hal_t;

/** Returns true when all mandatory callbacks are present. */
bool fcs_hal_validate(const fcs_hal_t *hal);

#ifdef __cplusplus
}
#endif

#endif /* FCS_HAL_H */
