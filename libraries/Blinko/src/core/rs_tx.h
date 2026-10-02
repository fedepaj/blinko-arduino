/*
 * rs_tx.h — Blinko transmitter v2: message slots, priority carousel, fountain
 * coding, visible-blink bursts, chip source. Driven from a timer ISR
 * (rs_tx_next_chip) or a bare-metal loop (fault handler). No allocation.
 */
#ifndef RS_TX_H
#define RS_TX_H

#include "rs_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_TX_MAX_FAULT_WEIGHT 4

typedef struct {
    uint8_t  valid;
    uint8_t  len;                 /* 1..31 (packed length if packed) */
    uint8_t  level;
    uint8_t  packed;              /* data[] holds rs_pack6 output */
    uint16_t next_seed;           /* rolling seed for the next visit (0 = systematic pass first) */
    uint32_t seq;                 /* recency counter */
    uint8_t  data[RS_MSG_MAX_LEN];
} rs_slot_t;

typedef struct {
    rs_slot_t slots[RS_NUM_SLOTS];
    uint32_t  seq_counter;
    uint8_t   next_log_slot;

    /* scheduler */
    uint8_t   round[RS_NUM_SLOTS * RS_TX_MAX_FAULT_WEIGHT];
    uint8_t   fault_weight;       /* FAULT visits per other visit in the round (1..RS_TX_MAX_FAULT_WEIGHT) */
    uint8_t   round_len;
    uint8_t   round_pos;
    rs_slot_t cur;                /* snapshot of the message being sent */
    uint8_t   cur_id;
    uint8_t   cur_sent;           /* packets sent in this visit; 0 = META next */
    uint8_t   visit_len;          /* packets per visit after META */

    /* visible-blink bursts, in chips (0 = continuous) */
    uint32_t  burst_on, burst_off;
    uint32_t  burst_pos;
    uint8_t   in_pause;

    /* channels: 1 (all LEDs same stream) or 3 (RGB, independent streams) */
    uint8_t   nchan;
    uint32_t  pilot_period;       /* mean chips between pilot blocks (RGB mode), 0 = none */
    uint32_t  pilot_next;         /* chips until the next block: the period jittered (see rs_tx.c) */
    uint32_t  pilot_count;        /* blocks sent, drives the jitter sequence */
    uint32_t  pilot_pos;
    uint8_t   in_pilot;
    uint32_t  pilot_idx;

    /* chip output, per channel */
    uint8_t   chips[RS_MAX_CHANNELS][RS_PKT_CHIPS];
    uint8_t   chip_pos[RS_MAX_CHANNELS];
    uint32_t  packets_sent;
    /* repetition: every packet is sent `repeat` times back to back (1 = off). A camera whose
     * window is shorter than a packet still reads a whole one across two copies (cyclic decode). */
    uint8_t   repeat;
    uint8_t   rep_left[RS_MAX_CHANNELS];
} rs_tx_t;

void rs_tx_init(rs_tx_t *tx);

/* Store a message in a rotating log slot; text is 6-bit packed when shorter.
 * Truncates to 31 (packed) bytes. Returns the slot id. Caller masks the chip ISR. */
uint8_t rs_tx_log(rs_tx_t *tx, uint8_t level, const char *text, size_t len);
void rs_tx_set_slot(rs_tx_t *tx, uint8_t id, uint8_t level, const char *text, size_t len);
void rs_tx_clear_slot(rs_tx_t *tx, uint8_t id);

/* Visible blink: transmit for on_chips, dark for off_chips (packet-aligned). */
void rs_tx_set_burst(rs_tx_t *tx, uint32_t on_chips, uint32_t off_chips);
/* Airtime of the FAULT slot: weight w inserts w-1 extra FAULT visits after every other visit
 * (w=1: FAULT once per round; w=3: about 85 % of the packets). Used by the death loop so the
 * fault reason completes in 1-2 s while the last logs still follow. */
void rs_tx_set_fault_weight(rs_tx_t *tx, uint8_t w);

/* Next (id, seed, payload) according to the carousel. */
void rs_tx_next_packet(rs_tx_t *tx, uint8_t *id, uint16_t *seed, uint8_t *payload);

/* RGB: 1 or 3 channels; pilot blocks every pilot_period chips (3-channel mode). */
void rs_tx_set_channels(rs_tx_t *tx, uint8_t nchan, uint32_t pilot_period);
/* Send every packet n times back to back (1..4; default 1). */
void rs_tx_set_repeat(rs_tx_t *tx, uint8_t n);   /* 1..100 copies of every packet */

/* Next chips for all channels (out[0..2], each 0/1). Call once per T_chip.
 * In 1-channel mode out[1] and out[2] mirror out[0]. */
void rs_tx_next_chips(rs_tx_t *tx, uint8_t out[RS_MAX_CHANNELS]);

/* Single-channel convenience: next chip of channel 0. */
uint8_t rs_tx_next_chip(rs_tx_t *tx);
/* rs_encode_packet as a linkable function (bindings, tests). */
int rs_tx_encode(uint8_t id, uint16_t seed, uint8_t payload, uint8_t *chips);

#ifdef __cplusplus
}
#endif
#endif
