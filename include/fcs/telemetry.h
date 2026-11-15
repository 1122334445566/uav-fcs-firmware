/**
 * @file telemetry.h
 * @brief Compact MAVLink-style binary link (framing + CRC), no external deps.
 *
 * Wire format (little endian):
 *   [0]     0xFE sync
 *   [1]     payload length (0..64)
 *   [2]     sequence
 *   [3]     message id
 *   [4..]   payload
 *   [-2..]  CRC16-CCITT over bytes [1 .. end of payload], low byte first
 *
 * This is intentionally NOT a full MAVLink implementation; docs/ARCHITECTURE.md
 * explains how to swap in real MAVLink by replacing telemetry.c only.
 */
#ifndef FCS_TELEMETRY_H
#define FCS_TELEMETRY_H

#include <stdint.h>
#include "fcs/fcs_types.h"
#include "fcs/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FCS_TLM_SYNC          0xFEu
#define FCS_TLM_HEADER_SIZE   4u
#define FCS_TLM_CRC_SIZE      2u
#define FCS_TLM_MAX_PAYLOAD   64u
#define FCS_TLM_MAX_FRAME     (FCS_TLM_HEADER_SIZE + FCS_TLM_MAX_PAYLOAD + FCS_TLM_CRC_SIZE)

typedef enum {
    FCS_TLM_HEARTBEAT = 1,
    FCS_TLM_ATTITUDE  = 2,
    FCS_TLM_POSITION  = 3,
    FCS_TLM_ACTUATORS = 4,
    FCS_TLM_STATUS    = 5
} fcs_tlm_msg_id_t;

typedef struct {
    uint8_t  seq;
    uint32_t frames_sent;
    uint32_t crc_errors;
} fcs_telemetry_t;

/* ---- codec (pure, testable without any hardware) ---- */
uint16_t fcs_crc16_ccitt(const uint8_t *data, uint16_t len, uint16_t seed);

/** Returns frame length, or 0 on invalid arguments. */
uint16_t fcs_telemetry_encode(uint8_t *frame, uint16_t capacity,
                              uint8_t msg_id, uint8_t seq,
                              const uint8_t *payload, uint8_t len);

/** Returns payload length, or -1 on bad length / bad CRC. */
int fcs_telemetry_decode(const uint8_t *frame, uint16_t frame_len,
                         uint8_t *msg_id, uint8_t *seq,
                         uint8_t *payload, uint8_t payload_capacity);

/* ---- little-endian payload writers ---- */
void fcs_tlm_put_u8(uint8_t *buf, uint8_t *offset, uint8_t v);
void fcs_tlm_put_u32(uint8_t *buf, uint8_t *offset, uint32_t v);
void fcs_tlm_put_f32(uint8_t *buf, uint8_t *offset, float v);
float fcs_tlm_get_f32(const uint8_t *buf, uint8_t *offset);
uint32_t fcs_tlm_get_u32(const uint8_t *buf, uint8_t *offset);

/* ---- high level senders ---- */
void fcs_telemetry_init(fcs_telemetry_t *t);

void fcs_telemetry_send_heartbeat(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                  uint32_t uptime_ms, fcs_arm_state_t arm,
                                  fcs_flight_mode_t mode, uint8_t fault,
                                  uint32_t loop_us);

void fcs_telemetry_send_attitude(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                 const fcs_state_t *state);

void fcs_telemetry_send_position(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                 const fcs_state_t *state);

void fcs_telemetry_send_actuators(fcs_telemetry_t *t, const fcs_hal_t *hal,
                                  const float *motors, uint8_t count, float thrust);

void fcs_telemetry_send_status(fcs_telemetry_t *t, const fcs_hal_t *hal,
                               const char *text);

#ifdef __cplusplus
}
#endif

#endif /* FCS_TELEMETRY_H */
