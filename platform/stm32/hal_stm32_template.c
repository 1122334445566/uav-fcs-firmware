/**
 * @file hal_stm32_template.c
 * @brief STM32 BSP skeleton - NOT compiled by the host CMake/Makefile.
 *
 * Copy this file into your STM32CubeIDE / PlatformIO project, replace every
 * `bsp_*` hook with real driver calls, then link against libfcs_core.a.
 * The `bsp_*` functions are declared (not defined) here on purpose: it keeps
 * this template free of any vendor SDK dependency while documenting exactly
 * what the board support package must provide.
 *
 * Target assumptions: Cortex-M4F, 1 kHz control loop, BMI270 + BMP388 +
 * UBLOX GNSS + SBUS/CRSF receiver, DShot600 ESCs.
 */
#include "fcs/fcs.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* BSP hooks YOU must implement (see docs/PORTING.md)                   */
/* ------------------------------------------------------------------ */
extern bool     bsp_imu_read(fcs_imu_t *out);          /* rad/s, m/s^2 */
extern bool     bsp_baro_read(fcs_baro_t *out);
extern bool     bsp_gnss_read(fcs_gnss_t *out);        /* local NED */
extern bool     bsp_rc_read(fcs_rc_t *out);
extern bool     bsp_battery_read(float *voltage_v);
extern void     bsp_motor_write(uint8_t index, float value);
extern uint64_t bsp_time_us(void);                     /* monotonic */
extern void     bsp_log(fcs_log_level_t level, const char *message);
extern void     bsp_telemetry_write(const uint8_t *bytes, uint16_t len);
extern bool     bsp_flash_read(uint32_t addr, void *dst, uint16_t size);
extern bool     bsp_flash_write(uint32_t addr, const void *src, uint16_t size);
extern void     bsp_watchdog_feed(void);

#define FLASH_PARAM_ADDR   0x080E0000u   /* last sector, adjust per device */
#define FLASH_PARAM_SIZE   2048u

/* ------------------------------------------------------------------ */
/* HAL callbacks                                                       */
/* ------------------------------------------------------------------ */
static bool hal_imu(fcs_imu_t *out, void *ctx)
{
    (void)ctx;
    return bsp_imu_read(out);
}

static bool hal_baro(fcs_baro_t *out, void *ctx)
{
    (void)ctx;
    return bsp_baro_read(out);
}

static bool hal_gnss(fcs_gnss_t *out, void *ctx)
{
    (void)ctx;
    return bsp_gnss_read(out);
}

static bool hal_rc(fcs_rc_t *out, void *ctx)
{
    (void)ctx;
    return bsp_rc_read(out);
}

static bool hal_battery(float *voltage_v, void *ctx)
{
    (void)ctx;
    return bsp_battery_read(voltage_v);
}

static void hal_motor_write(const float *outputs, uint8_t count, void *ctx)
{
    (void)ctx;
    if (count > 4u) count = 4u;
    for (uint8_t i = 0; i < count; ++i) {
        /* NaN / negative guard: never forward an invalid throttle to an ESC. */
        float v = outputs[i];
        if (!(v > 0.0f)) v = 0.0f;
        if (v > 1.0f)    v = 1.0f;
        /* TODO: map v to DShot frame 48..2047 or PWM 1000..2000 us.
         * The motor ORDER must match fcs_xquad_matrix in src/control/mixer.c:
         *   0 = front-right CCW, 1 = rear-right CW,
         *   2 = rear-left  CCW, 3 = front-left CW.                       */
        bsp_motor_write(i, v);
    }
}

static uint64_t hal_time_us(void *ctx)
{
    (void)ctx;
    return bsp_time_us();
}

static void hal_log(fcs_log_level_t level, const char *message, void *ctx)
{
    (void)ctx;
    bsp_log(level, message);
}

static void hal_telemetry(const uint8_t *bytes, uint16_t len, void *ctx)
{
    (void)ctx;
    bsp_telemetry_write(bytes, len);   /* must be DMA / ring buffered */
}

static bool hal_param_load(void *buffer, uint16_t size, void *ctx)
{
    (void)ctx;
    if (size > FLASH_PARAM_SIZE) return false;
    /* TODO: verify a CRC / A-B image before accepting the blob. */
    return bsp_flash_read(FLASH_PARAM_ADDR, buffer, size);
}

static bool hal_param_save(const void *buffer, uint16_t size, void *ctx)
{
    (void)ctx;
    if (size > FLASH_PARAM_SIZE) return false;
    /* TODO: erase the sector and write an A/B redundant copy + CRC. */
    return bsp_flash_write(FLASH_PARAM_ADDR, buffer, size);
}

static void hal_watchdog(void *ctx)
{
    (void)ctx;
    bsp_watchdog_feed();
}

fcs_hal_t fcs_stm32_hal(void)
{
    fcs_hal_t hal;
    memset(&hal, 0, sizeof(hal));
    hal.imu_read        = hal_imu;
    hal.baro_read       = hal_baro;
    hal.gnss_read       = hal_gnss;
    hal.rc_read         = hal_rc;
    hal.battery_read    = hal_battery;
    hal.motor_write     = hal_motor_write;
    hal.time_us         = hal_time_us;
    hal.log             = hal_log;
    hal.telemetry_write = hal_telemetry;
    hal.param_load      = hal_param_load;
    hal.param_save      = hal_param_save;
    hal.watchdog_feed   = hal_watchdog;
    hal.ctx             = NULL;
    return hal;
}

/* ------------------------------------------------------------------ */
/* Application skeleton (copy into main.c)                              */
/* ------------------------------------------------------------------ */
#if 0
static fcs_t g_fcs;
static volatile uint64_t g_last_tick_us;

void app_flight_control_init(void)
{
    fcs_hal_t hal = fcs_stm32_hal();
    /* TODO: 2 s ESC calibration with all outputs forced to zero first. */
    if (!fcs_init(&g_fcs, &hal, NULL)) {
        /* halt: invalid HAL table, never take off */
        for (;;) { }
    }
    g_last_tick_us = bsp_time_us();
}

/* Called from a 1 kHz TIM interrupt, highest priority after IMU DRDY. */
void app_flight_control_tick(void)
{
    uint64_t now = bsp_time_us();
    if (now - g_last_tick_us < 1000u) return;
    g_last_tick_us = now;
    fcs_step(&g_fcs, now);
}

/* Ground station may call these from a low-priority task. */
void app_set_mission(const fcs_waypoint_t *wps, uint8_t count)
{
    (void)fcs_set_waypoints(&g_fcs, wps, count);
    fcs_request_mode(&g_fcs, FCS_MODE_AUTO);
}
#endif
