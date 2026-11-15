#include "sim.h"

#include <stdio.h>
#include <string.h>

static bool imu_read(fcs_imu_t *out, void *ctx)
{
    fcs_sim_t *s = (fcs_sim_t *)ctx;
    if (!s->imu_available) return false;
    fcs_sim_sample_imu(s, out);
    return true;
}

static bool baro_read(fcs_baro_t *out, void *ctx)
{
    fcs_sim_sample_baro((fcs_sim_t *)ctx, out);
    return true;
}

static bool gnss_read(fcs_gnss_t *out, void *ctx)
{
    fcs_sim_t *s = (fcs_sim_t *)ctx;
    fcs_sim_sample_gnss(s, out);
    return s->gnss_available;
}

static bool rc_read(fcs_rc_t *out, void *ctx)
{
    fcs_sim_t *s = (fcs_sim_t *)ctx;
    if (!s->rc_available) return false;
    *out = s->rc;
    return true;
}

static bool battery_read(float *voltage_v, void *ctx)
{
    *voltage_v = ((fcs_sim_t *)ctx)->battery_v;
    return true;
}

static void motor_write(const float *outputs, uint8_t count, void *ctx)
{
    fcs_sim_write_motors((fcs_sim_t *)ctx, outputs, count);
}

static uint64_t time_us(void *ctx)
{
    return ((fcs_sim_t *)ctx)->time_us;
}

static void log_line(fcs_log_level_t level, const char *message, void *ctx)
{
    fcs_sim_t *s = (fcs_sim_t *)ctx;
    if (!s->verbose) return;
    const char *tag = (level == FCS_LOG_ERROR) ? "ERROR" :
                      (level == FCS_LOG_WARN)  ? "WARN " :
                      (level == FCS_LOG_INFO)  ? "INFO " : "DEBUG";
    fprintf(stderr, "[%s] %s\n", tag, message);
}

static void telemetry_write(const uint8_t *bytes, uint16_t len, void *ctx)
{
    fcs_sim_t *s = (fcs_sim_t *)ctx;
    (void)bytes;
    s->telemetry_bytes += len;
}

static bool param_load(void *buffer, uint16_t size, void *ctx)
{
    (void)ctx;
    FILE *fp = fopen("fcs_params.bin", "rb");
    if (!fp) return false;
    size_t n = fread(buffer, 1, size, fp);
    fclose(fp);
    return n == size;
}

static bool param_save(const void *buffer, uint16_t size, void *ctx)
{
    (void)ctx;
    FILE *fp = fopen("fcs_params.bin", "wb");
    if (!fp) return false;
    size_t n = fwrite(buffer, 1, size, fp);
    fclose(fp);
    return n == size;
}

static void watchdog_feed(void *ctx)
{
    ((fcs_sim_t *)ctx)->watchdog_feeds++;
}

fcs_hal_t fcs_sim_hal(fcs_sim_t *s)
{
    fcs_hal_t h;
    memset(&h, 0, sizeof(h));
    h.imu_read        = imu_read;
    h.baro_read       = baro_read;
    h.gnss_read       = gnss_read;
    h.rc_read         = rc_read;
    h.battery_read    = battery_read;
    h.motor_write     = motor_write;
    h.time_us         = time_us;
    h.log             = log_line;
    h.telemetry_write = telemetry_write;
    h.param_load      = param_load;
    h.param_save      = param_save;
    h.watchdog_feed   = watchdog_feed;
    h.ctx             = s;
    return h;
}
