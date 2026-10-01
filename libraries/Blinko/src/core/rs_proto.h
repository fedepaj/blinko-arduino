/*
 * rs_proto.h — Blinko optical protocol v3: shared definitions (encoder + decoder).
 * Freestanding C99, no allocation, no libc except stdint/stddef.
 * See docs/PROTOCOL.md for the wire format as it is; this header keeps the reasons.
 *
 * Time unit: the chip (= one code cell, the transmitter's timer period). The minimum run of
 * the line code is T = RS_CELLS_PER_T chips: T is what the camera exposure must not exceed,
 * so a board is configured by T and its timer runs at T / 3.
 *
 * Line code: RLL(2,7) in NRZI (transitions are 3..8 chips apart, 1.5 bits per T), a
 * variable-length table code driven bit by bit (rs_rll27_step). A run of RS_SYNC_ON_CHIPS
 * cannot occur in data, so the sync is a run-length violation.
 *
 * Packet (82 chips): sync [off 3][on 10][off 3] then RS_DATA_CHIPS chips carrying 30 bits:
 *   id(3) seed(7) payload(8) crc12(12), encoded from level OFF; the tree is flushed with zero
 *   bits and the field is filled with runs of 4 chips, so every packet has the same length
 *   and packets sit on a grid.
 *   seed 127      META: payload = [len:5][level:3], message stored as raw bytes
 *   seed 126      META: same, message bytes are 6-bit packed text (rs_pack.h)
 *   seed 125      MSGCRC:  payload = CRC-8/ATM of the message bytes   (guard the fountain decode:
 *   seed 124      MSGCRC2: payload = CRC-8/MAXIM of the message bytes  16 bits total)
 *   seed < len    systematic: payload = message byte[seed]
 *   len <= seed < 124  coded: payload = XOR of message bytes selected by rs_code_mask(seed, len)
 *
 * Why v3 (October 2026) looks like this — v2 was Manchester (bit 1 = 01, bit 0 = 10), a
 * [0][1111][0000][10] sync, 67-chip packets with id(3) seed(9) payload(8) CRC-8 (poly 0x07,
 * init 0) at T_chip = 30 us, and a threshold decoder. It did 45-120 packets/s on an iPhone 14
 * and zero on the first Android phone tested (Samsung S21 FE), for three reasons:
 *   1. its shortest exposure is 57.5 us = 1.9 Manchester chips. A sensor row is a box filter
 *      over the exposure, so a 1-chip run at E = 1.9 T is attenuated to almost nothing. The
 *      code needed longer minimum runs, so the exposure is set against the run, not the chip:
 *      RLL(2,7) keeps every run >= 3 chips and still carries 1.5 bits per minimum run
 *      (Manchester 1, Miller 1, 8B10B 0.8). On the simulator at the Samsung's numbers
 *      (T = 60 us, E/T = 1) it decoded 0.70 packets/frame vs 0.17 Miller and 0 Manchester
 *      (core/tools/codelab.py also tried NRZ, 4B6B, 8B10B, PPM). Scrambled 64b/66b-style codes
 *      were rejected: no run-length guarantee, and they need a long synchronous stream while a
 *      rolling shutter reads a few milliseconds at a time.
 *   2. its readout is 6-8.5 ms of a 33 ms frame and the LED blob covered 1.3-4.5 ms of it: a
 *      2 ms packet rarely fit whole. Hence the shorter-in-time packet for the same bits, the
 *      repetition option (rs_tx_set_repeat) read across two copies by the receiver's cyclic
 *      decode, and the receiver's backward decode (the packet before a sync).
 *   3. its ISP compresses highlights (luma ceiling ~238), hiding clipped stripes: the Android
 *      app got a RAW capture path; not a protocol matter but part of the same diagnosis.
 * CRC-12 with a non-zero init replaced CRC-8/init 0 because the new receiver tries many
 * hypotheses per frame (~1/256 false accepts per completed trial with CRC-8) and because an
 * all-zero packet, which a falling blob edge decodes to, passed the old CRC. The seed lost 2
 * bits (messages are <= 31 bytes, 124 seeds suffice) and the start bit went (the 10-chip sync
 * needs none). The 10-chip sync replaced 4+4: two edges 10 chips apart give the clock to ~1 %
 * with smeared edges, and OFF runs of 3 on both sides are legal neighbours of the data.
 */
#ifndef RS_PROTO_H
#define RS_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS_PROTO_VERSION      3

#define RS_CELLS_PER_T        3      /* chips per minimum run: RLL(2,7) */
#define RS_SYNC_GAP_CHIPS     3
#define RS_SYNC_ON_CHIPS      10
#define RS_SYNC_OFF_CHIPS     3
#define RS_SYNC_CHIPS         (RS_SYNC_GAP_CHIPS + RS_SYNC_ON_CHIPS + RS_SYNC_OFF_CHIPS)   /* 16 */
#define RS_ID_BITS            3
#define RS_SEED_BITS          7
#define RS_HDR_BITS           (RS_ID_BITS + RS_SEED_BITS)                                  /* 10 */
#define RS_PAYLOAD_BITS       8
#define RS_CRC_BITS           12
#define RS_DATA_BITS          (RS_HDR_BITS + RS_PAYLOAD_BITS + RS_CRC_BITS)                /* 30 */
#define RS_DATA_CHIPS         (2 * (RS_DATA_BITS + 3))                                     /* 66: rate 1/2 + up to 3 flush bits */
#define RS_PKT_CHIPS          (RS_SYNC_CHIPS + RS_DATA_CHIPS)                              /* 82 */
#define RS_RLL_MIN_RUN        3      /* chips between transitions, inclusive bounds */
#define RS_RLL_MAX_RUN        8

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

#define RS_SEED_META          127
#define RS_SEED_META_PACKED   126
#define RS_SEED_MSGCRC        125
#define RS_SEED_MSGCRC2       124
#define RS_SEED_CODED_MAX     123
#define RS_SEED_MASK          127

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

/* CRC-12 (poly 0x80F, init 0xFFF, MSB first) over the nbits low bits of v. A non-zero init
 * rejects the all-zero packet that a dark-to-bright blob edge decodes to. */
static inline uint16_t rs_crc12_bits(uint32_t v, int nbits)
{
    uint16_t crc = 0xFFF;
    for (int i = nbits - 1; i >= 0; i--) {
        uint16_t in = (uint16_t)(((v >> i) & 1u) ^ ((crc >> 11) & 1u));
        crc = (uint16_t)((crc << 1) & 0xFFFu);
        if (in) crc ^= 0x80Fu;
    }
    return crc;
}

static inline uint32_t rs_fields(uint8_t id, uint16_t seed, uint8_t payload)
{
    return ((uint32_t)(id & 7u) << (RS_SEED_BITS + RS_PAYLOAD_BITS)) | ((uint32_t)(seed & RS_SEED_MASK) << RS_PAYLOAD_BITS) | payload;  /* 18 bits */
}

static inline uint16_t rs_crc_fields(uint8_t id, uint16_t seed, uint8_t payload)
{
    return rs_crc12_bits(rs_fields(id, seed, payload), RS_HDR_BITS + RS_PAYLOAD_BITS);
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

/* RLL(2,7) encoder state machine. Nodes of the prefix tree: 0 root, 1 "0", 2 "1", 3 "00",
 * 4 "01", 5 "001". Feeding a bit moves down the tree; at a leaf the codeword is emitted as
 * NRZI chips (codeword '1' = transition of *level) and the node returns to the root.
 * Codewords: 10->0100 11->1000 000->000100 010->100100 011->001000 0010->00100100 0011->00001000 */
static inline uint8_t rs_rll27_step(uint8_t node, uint8_t bit, uint8_t *level, uint8_t *chips, int *nchips)
{
    static const uint8_t next[6][2]  = { {1, 2}, {3, 4}, {0, 0}, {0, 5}, {0, 0}, {0, 0} };
    static const uint8_t wlen[6][2]  = { {0, 0}, {0, 0}, {4, 4}, {6, 0}, {6, 6}, {8, 8} };
    static const uint8_t words[6][2] = { {0, 0}, {0, 0}, {0x4, 0x8}, {0x04, 0}, {0x24, 0x08}, {0x24, 0x08} };  /* codeword bits, MSB first */
    *nchips = 0;
    int len = wlen[node][bit];
    if (len == 0) return next[node][bit];
    uint32_t w = words[node][bit];
    for (int i = len - 1; i >= 0; i--) {
        if ((w >> i) & 1u) *level ^= 1u;
        chips[(*nchips)++] = *level;
    }
    return 0;
}

/* Encode one packet into RS_PKT_CHIPS chips (each 0 or 1). Returns RS_PKT_CHIPS. */
static inline int rs_encode_packet(uint8_t id, uint16_t seed, uint8_t payload, uint8_t *chips)
{
    uint32_t bits = (rs_fields(id, seed, payload) << RS_CRC_BITS) | rs_crc_fields(id, seed, payload);   /* 30 bits */
    int n = 0;
    for (int i = 0; i < RS_SYNC_GAP_CHIPS; i++) chips[n++] = 0;
    for (int i = 0; i < RS_SYNC_ON_CHIPS; i++) chips[n++] = 1;
    for (int i = 0; i < RS_SYNC_OFF_CHIPS; i++) chips[n++] = 0;
    uint8_t node = 0, level = 0, buf[8]; int k;
    for (int b = RS_DATA_BITS - 1; b >= 0; b--) {
        node = rs_rll27_step(node, (uint8_t)((bits >> b) & 1u), &level, buf, &k);
        for (int i = 0; i < k; i++) chips[n++] = buf[i];
    }
    while (node != 0) {                              /* flush the tree with zero bits */
        node = rs_rll27_step(node, 0, &level, buf, &k);
        for (int i = 0; i < k; i++) chips[n++] = buf[i];
    }
    /* filler to a constant length: runs of 4 chips (never a run of RS_SYNC_ON_CHIPS, and the
     * last codeword always ends with a run >= 3, so the (2,7) constraint holds throughout) */
    int run = 0;
    for (int i = n - 1; i >= RS_SYNC_CHIPS && chips[i] == level; i--) run++;
    while (n < RS_PKT_CHIPS) {
        if (run >= 4) { level ^= 1u; run = 0; }
        chips[n++] = level; run++;
    }
    return n;
}

#ifdef __cplusplus
}
#endif
#endif /* RS_PROTO_H */
