#include "fcs/telemetry.h"

#include <stdio.h>
#include <string.h>

uint16_t fcs_crc16_ccitt(const uint8_t *data, uint16_t len, uint16_t seed)
{
    uint16_t crc = seed;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t fcs_telemetry_encode(uint8_t *frame, uint16_t capacity,
                              uint8_t msg_id, uint8_t seq,
                              const uint8_t *payload, uint8_t len)
{
    uint16_t total = FCS_TLM_HEADER_SIZE + len + FCS_TLM_CRC_SIZE;
    if (!frame || len > FCS_TLM_MAX_PAYLOAD || capacity < total) return 0u;
    if (len > 0u && !payload) return 0u;

    frame[0] = FCS_TLM_SYNC;
    frame[1] = len;
    frame[2] = seq;
    frame[3] = msg_id;
    if (len > 0u) memcpy(&frame[4], payload, len);

    uint16_t crc = fcs_crc16_ccitt(&frame[1], (uint16_t)(3u + len), 0xFFFFu);
    frame[4 + len]     = (uint8_t)(crc & 0xFFu);
    frame[4 + len + 1] = (uint8_t)(crc >> 8);
    return total;
}

int fcs_telemetry_decode(const uint8_t *frame, uint16_t frame_len,
                         uint8_t *msg_id, uint8_t *seq,
                         uint8_t *payload, uint8_t payload_capacity)
{
    if (!frame || frame_len < FCS_TLM_HEADER_SIZE + FCS_TLM_CRC_SIZE) return -1;
    if (frame[0] != FCS_TLM_SYNC) return -1;
    uint8_t len = frame[1];
    if (len > FCS_TLM_MAX_PAYLOAD) return -1;
    if (frame_len != FCS_TLM_HEADER_SIZE + len + FCS_TLM_CRC_SIZE) return -1;

    uint16_t expected = fcs_crc16_ccitt(&frame[1], (uint16_t)(3u + len), 0xFFFFu);
    uint16_t got = (uint16_t)(frame[4 + len] | ((uint16_t)frame[5 + len] << 8));
    if (got != expected) return -1;
    if (len > payload_capacity) return -1;

    if (msg_id) *msg_id = frame[3];
    if (seq)    *seq = frame[2];
    if (len > 0u && payload) memcpy(payload, &frame[4], len);
    return (int)len;
}

static void put_u32_le(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)(v & 0xFFu);
    b[1] = (uint8_t)((v >> 8) & 0xFFu);
    b[2] = (uint8_t)((v >> 16) & 0xFFu);
    b[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32_le(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8)
         | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

void fcs_tlm_put_u8(uint8_t *buf, uint8_t *offset, uint8_t v)
{ buf[(*offset)++] = v; }

void fcs_tlm_put_u32(uint8_t *buf, uint8_t *offset, uint32_t v)
{ put_u32_le(&buf[*offset], v); *offset += 4u; }

void fcs_tlm_put_f32(uint8_t *buf, uint8_t *offset, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    put_u32_le(&buf[*offset], u);
    *offset += 4u;
}

uint32_t fcs_tlm_get_u32(const uint8_t *buf, uint8_t *offset)
{
    uint32_t v = get_u32_le(&buf[*offset]);
    *offset += 4u;
    return v;
}

float fcs_tlm_get_f32(const uint8_t *buf, uint8_t *offset)
{
    uint32_t u = get_u32_le(&buf[*offset]);
    *offset += 4u;
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static void send(fcs_telemetry_t *t, const fcs_hal_t *hal,
                 uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[FCS_TLM_MAX_FRAME];
    uint16_t n = fcs_telemetry_encode(frame, (uint16_t)sizeof(frame),
                                      msg_id, t->seq, payload, len);
    if (n == 0u) return;
    t->seq++;
    t->frames_sent++;
    if (hal->telemetry_write) hal->telemetry_write(frame, (uint16_t)n, hal->ctx);
}

void fcs_telemetry_init(fcs_telemetry_t *t)
{
    t->seq = 0u;
    t->frames_sent = 0u;
    t->crc_errors = 0u;
}

void fcs_telemetry_send_heartbeat(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                  uint32_t uptime_ms, fcs_arm_state_t arm,
                                  fcs_flight_mode_t mode, uint8_t fault,
                                  uint32_t loop_us)
{
    uint8_t b[12], o = 0u;
    fcs_tlm_put_u32(b, &o, uptime_ms);
    fcs_tlm_put_u8(b, &o, (uint8_t)arm);
    fcs_tlm_put_u8(b, &o, (uint8_t)mode);
    fcs_tlm_put_u8(b, &o, fault);
    fcs_tlm_put_u8(b, &o, 0u);
    fcs_tlm_put_u32(b, &o, loop_us);
    send(t, hal, (uint8_t)FCS_TLM_HEARTBEAT, b, o);
}

void fcs_telemetry_send_attitude(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                 const fcs_state_t *state)
{
    uint8_t b[24], o = 0u;
    fcs_tlm_put_f32(b, &o, state->euler.roll);
    fcs_tlm_put_f32(b, &o, state->euler.pitch);
    fcs_tlm_put_f32(b, &o, state->euler.yaw);
    fcs_tlm_put_f32(b, &o, state->rates.x);
    fcs_tlm_put_f32(b, &o, state->rates.y);
    fcs_tlm_put_f32(b, &o, state->rates.z);
    send(t, hal, (uint8_t)FCS_TLM_ATTITUDE, b, o);
}

void fcs_telemetry_send_position(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                 const fcs_state_t *state)
{
    uint8_t b[28], o = 0u;
    fcs_tlm_put_f32(b, &o, state->pos_ned.x);
    fcs_tlm_put_f32(b, &o, state->pos_ned.y);
    fcs_tlm_put_f32(b, &o, state->pos_ned.z);
    fcs_tlm_put_f32(b, &o, state->vel_ned.x);
    fcs_tlm_put_f32(b, &o, state->vel_ned.y);
    fcs_tlm_put_f32(b, &o, state->vel_ned.z);
    fcs_tlm_put_f32(b, &o, state->altitude_m);
    send(t, hal, (uint8_t)FCS_TLM_POSITION, b, o);
}

void fcs_telemetry_send_actuators(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                  const float *motors, uint8_t count, float thrust)
{
    uint8_t b[24], o = 0u;
    if (count > 4u) count = 4u;
    fcs_tlm_put_u8(b, &o, count);
    for (uint8_t i = 0; i < 4u; ++i)
        fcs_tlm_put_f32(b, &o, (i < count) ? motors[i] : 0.0f);
    fcs_tlm_put_f32(b, &o, thrust);
    send(t, hal, (uint8_t)FCS_TLM_ACTUATORS, b, o);
}

void fcs_telemetry_send_status(fcs_telemetry_t *t, const fcs_hal_t *hal,
                               const char *text)
{
    uint8_t b[FCS_TLM_MAX_PAYLOAD];
    memset(b, 0, sizeof(b));
    if (text) {
        size_t n = strlen(text);
        if (n > sizeof(b) - 1u) n = sizeof(b) - 1u;
        memcpy(b, text, n);
    }
    send(t, hal, (uint8_t)FCS_TLM_STATUS, b, (uint8_t)sizeof(b));
}
