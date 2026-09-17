/*
 * rs_proto.h — RSLog optical protocol v2: shared definitions (encoder + decoder).
 * Freestanding C99, no allocation, no libc except stdint/stddef.
 * See docs/PROTOCOL.md.
 *
 * Packet (67 chips): gap(1) sync(8: 1111 0000) start(2: bit 0) then 28
 * Manchester bits: id(3) seed(9) payload(8) crc8(8).
 *   seed 511      META: payload = [len:5][level:3], message stored as raw bytes
 *   seed 510      META: same, message bytes are 6-bit packed text (rs_pack.h)
 *   seed 509      MSGCRC:  payload = CRC-8/ATM of the message bytes   (guard the fountain decode:
 *   seed 508      MSGCRC2: payload = CRC-8/MAXIM of the message bytes  16 bits total)
 *   seed < len    systematic: payload = message byte[seed]
 *   len <= seed < 508  coded: payload = XOR of message bytes selected by rs_code_mask(seed, len)
 */
#ifndef RS_PROTO_H
#define RS_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS_PROTO_VERSION      2

#define RS_GAP_CHIPS          1
#define RS_SYNC_CHIPS         8
#define RS_START_CHIPS        2
#define RS_HDR_BITS           12     /* id:3 seed:9 */
#define RS_PAYLOAD_BITS       8
#define RS_CRC_BITS           8
#define RS_DATA_BITS          (RS_HDR_BITS + RS_PAYLOAD_BITS + RS_CRC_BITS)   /* 28 */
#define RS_PKT_BITS           (1 + RS_DATA_BITS)                                /* start bit + 28 = 29 */
#define RS_PKT_CHIPS          (RS_GAP_CHIPS + RS_SYNC_CHIPS + RS_START_CHIPS + 2 * RS_DATA_BITS) /* 67 */

#define RS_NUM_SLOTS          8
#define RS_NUM_LOG_SLOTS      6      /* slots 0..5 rotate */
#define RS_SLOT_STATUS        6
#define RS_SLOT_FAULT         7
#define RS_MSG_MAX_LEN        31
/* Multi-channel (RGB) mode: three independent packet streams on LEDs 0..2.
 * Every RS_PILOT_PERIOD chips the transmitter emits a pilot block so the
 * receiver can measure the camera's colour response to each LED:
 * [dark 2P][ch0 P][dark P][ch1 P][dark P][ch2 P][dark 2P], P = RS_PILOT_P. */
#define RS_MAX_CHANNELS       3
#define RS_PILOT_P            4     /* 4 chips per pulse: the block (9P = 36 chips) must fit inside the LED blob */
#define RS_PILOT_CHIPS        (9 * RS_PILOT_P)

#define RS_SEED_META          511
#define RS_SEED_META_PACKED   510
#define RS_SEED_MSGCRC        509
#define RS_SEED_MSGCRC2       508
#define RS_SEED_CODED_MAX     507

typedef enum {
    RS_LVL_DEBUG  = 0,
    RS_LVL_INFO   = 1,
    RS_LVL_WARN   = 2,
    RS_LVL_ERROR  = 3,
    RS_LVL_FATAL  = 4,
    RS_LVL_STATUS = 5,
    RS_LVL_FAULT  = 6,
    RS_LVL_RESERVED = 7
} rs_level_t;

static inline uint8_t rs_meta(uint8_t len, uint8_t level)   { return (uint8_t)(((len & 31u) << 3) | (level & 7u)); }
static inline uint8_t rs_meta_len(uint8_t meta)             { return (uint8_t)(meta >> 3); }
static inline uint8_t rs_meta_level(uint8_t meta)           { return (uint8_t)(meta & 7u); }
static inline int     rs_seed_is_meta(uint16_t seed)        { return seed >= RS_SEED_META_PACKED; }

/* CRC-8/ATM: poly 0x07, init 0x00. */
static inline uint8_t rs_crc8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80u) ? ((crc << 1) ^ 0x07u) : (crc << 1));
    }
    return crc;
}

/* CRC-8/MAXIM (reflected poly 0x8C, init 0x00): second, independent message check. */
static inline uint8_t rs_crc8b(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 1u) ? ((crc >> 1) ^ 0x8Cu) : (crc >> 1));
    }
    return crc;
}

/* The 20 header+payload bits as 3 bytes (last nibble zero) for the CRC. */
static inline void rs_fields_to_bytes(uint8_t id, uint16_t seed, uint8_t payload, uint8_t out[3])
{
    uint32_t v = ((uint32_t)(id & 7u) << 17) | ((uint32_t)(seed & 511u) << 8) | payload; /* 20 bits */
    out[0] = (uint8_t)(v >> 12);
    out[1] = (uint8_t)(v >> 4);
    out[2] = (uint8_t)((v & 0xFu) << 4);
}

static inline uint8_t rs_crc_fields(uint8_t id, uint16_t seed, uint8_t payload)
{
    uint8_t b[3];
    rs_fields_to_bytes(id, seed, payload, b);
    return rs_crc8(b, 3);
}

/* Pseudo-random subset of the message bytes for coded packet `seed` (len bytes).
 * Identical on every platform: xorshift32 seeded from (seed, len). Never zero. */
static inline uint32_t rs_code_mask(uint16_t seed, uint8_t len)
{
    uint32_t x = (uint32_t)seed * 2654435761u ^ ((uint32_t)len << 24) ^ 0x9E3779B9u;
    if (x == 0) x = 1;
    for (int i = 0; i < 3; i++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; }
    uint32_t full = (len >= 32) ? 0xFFFFFFFFu : ((1u << len) - 1u);
    uint32_t m = x & full;
    if (m == 0) m = 1u << (seed % (len ? len : 1));
    return m;
}

/* Encode one packet into RS_PKT_CHIPS chips (each 0 or 1). Returns RS_PKT_CHIPS. */
static inline int rs_encode_packet(uint8_t id, uint16_t seed, uint8_t payload, uint8_t *chips)
{
    uint32_t bits = ((uint32_t)(id & 7u) << 25) | ((uint32_t)(seed & 511u) << 16) |
                    ((uint32_t)payload << 8) | rs_crc_fields(id, seed, payload);   /* 28 bits */
    int n = 0;
    chips[n++] = 0;                                  /* gap */
    for (int i = 0; i < 4; i++) chips[n++] = 1;      /* sync high */
    for (int i = 0; i < 4; i++) chips[n++] = 0;      /* sync low */
    chips[n++] = 1; chips[n++] = 0;                  /* start bit (0) */
    for (int b = RS_DATA_BITS - 1; b >= 0; b--) {
        uint8_t bit = (uint8_t)((bits >> b) & 1u);
        chips[n++] = bit ? 0 : 1;                    /* 1 -> 01, 0 -> 10 */
        chips[n++] = bit ? 1 : 0;
    }
    return n;
}

#ifdef __cplusplus
}
#endif
#endif /* RS_PROTO_H */
