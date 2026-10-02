/*
 * rs_tx.h — Blinko transmitter: message slots, priority carousel, fountain
 * coding, visible-blink bursts, chip source. Driven from a timer ISR
 * (rs_tx_next_chips) or a bare-metal loop (fault handler). No allocation, no libc.
 *
 * Timing: rs_tx_next_chips is the per-chip path and is short. What takes long, choosing and
 * encoding the next packets (thousands of instructions, several chip periods at T = 60 us on a
 * 48 MHz core), is rs_tx_prepare: it fills a second buffer that the chip path swaps in at the
 * packet boundary. Call it from a context the chip interrupt can preempt (a lower-priority
 * interrupt such as PendSV, or the bare loop of a fault handler), any time rs_tx_wants_prepare
 * says so; a port that never calls it still works, the chip path then prepares at the boundary
 * itself, as it always did, and holds the LED for as long as that takes (measured on the Nano
 * R4: 8 % of the chip interrupts lost at T = 60 us, every packet 89 chips long instead of 82).
 *
 * Concurrency: the transmitter has no lock. Whoever calls a function that changes it
 * (rs_tx_put_slot, rs_tx_log_slot, rs_tx_clear_slot, rs_tx_set_*) while a timer ISR calls
 * rs_tx_next_chips masks that ISR, and whatever runs rs_tx_prepare, around the call. The expensive part of a new message, packing
 * its text, needs no masking: rs_tx_slot_prepare works on a slot of the caller's.
 */
#ifndef RS_TX_H
#define RS_TX_H

#include "rs_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_TX_MAX_FAULT_WEIGHT 4
#define RS_TX_MAX_REPEAT       100

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

    /* chip output, per channel: the packets on air in chips[air], the next ones in the other */
    uint8_t   chips[2][RS_MAX_CHANNELS][RS_PKT_CHIPS];
    uint8_t   air;
    volatile uint8_t next_ready;  /* the other buffer holds the next packets (rs_tx_prepare) */
    volatile uint8_t preparing;   /* rs_tx_prepare is filling it */
    uint8_t   chip_pos[RS_MAX_CHANNELS];
    uint32_t  packets_sent;
    /* repetition: every packet is sent `repeat` times back to back (1 = off). A camera whose
     * window is shorter than a packet still reads a whole one across two copies (cyclic decode). */
    uint8_t   repeat;
    uint8_t   rep_left;           /* copies of the packets on air still to send */
} rs_tx_t;

void rs_tx_init(rs_tx_t *tx);

/* A new message in two steps, so that the packing does not run with the chip ISR masked (it
 * costs thousands of instructions, several chip periods, and used to be done under the mask on
 * every log line).
 * 1. rs_tx_slot_prepare, ISR running: fills a slot of the caller's with the start of text —
 *    6-bit packed when that carries more of it (up to 41 characters in the 31 bytes) or the same
 *    in fewer bytes, raw otherwise — and returns how many characters it took. Call it again
 *    with the rest for a longer text. An empty text gives an invalid slot.
 * 2. rs_tx_log_slot / rs_tx_put_slot, ISR masked: a 44-byte copy into the next rotating log
 *    slot (returns its id, or RS_NUM_SLOTS for an invalid slot, which takes none) or into slot
 *    id (an invalid slot clears it). */
size_t  rs_tx_slot_prepare(rs_slot_t *s, uint8_t level, const char *text, size_t len);
uint8_t rs_tx_log_slot(rs_tx_t *tx, const rs_slot_t *s);
void    rs_tx_put_slot(rs_tx_t *tx, uint8_t id, const rs_slot_t *s);
/* Both steps in one call (text beyond one message is dropped): for tests and for callers
 * without an ISR. */
uint8_t rs_tx_log(rs_tx_t *tx, uint8_t level, const char *text, size_t len);
void rs_tx_set_slot(rs_tx_t *tx, uint8_t id, uint8_t level, const char *text, size_t len);
void rs_tx_clear_slot(rs_tx_t *tx, uint8_t id);

/* Visible blink: transmit for on_chips, dark for off_chips (packet-aligned). */
void rs_tx_set_burst(rs_tx_t *tx, uint32_t on_chips, uint32_t off_chips);
/* Airtime of the FAULT slot: weight w inserts w-1 extra FAULT visits after every other visit
 * (w=1: FAULT once per round; w=3: about three quarters of the packets). Used by the death loop so the
 * fault reason completes in 1-2 s while the last logs still follow. */
void rs_tx_set_fault_weight(rs_tx_t *tx, uint8_t w);

/* Next (id, seed, payload) according to the carousel. */
void rs_tx_next_packet(rs_tx_t *tx, uint8_t *id, uint16_t *seed, uint8_t *payload);

/* RGB: 1 or 3 channels; pilot blocks every pilot_period chips on average (3-channel mode).
 * The packet on air is cut short: every channel restarts at a packet boundary. */
void rs_tx_set_channels(rs_tx_t *tx, uint8_t nchan, uint32_t pilot_period);
/* Send every packet n times back to back (1..RS_TX_MAX_REPEAT, default 1): 2-3 for a camera
 * whose window is about one packet, tens for stitching across frames (rs_stitch.h). */
void rs_tx_set_repeat(rs_tx_t *tx, uint8_t n);

/* Next chips for all channels (out[0..2], each 0/1). Call once per chip period (T / 3).
 * In 1-channel mode out[1] and out[2] mirror out[0]. */
void rs_tx_next_chips(rs_tx_t *tx, uint8_t out[RS_MAX_CHANNELS]);
/* Choose and encode the packets that follow the ones on air (see Timing above). Does nothing
 * when they are ready. The packet boundary waits, LEDs dark, for a call the chip interrupt
 * preempted half way. */
void rs_tx_prepare(rs_tx_t *tx);
static inline int rs_tx_wants_prepare(const rs_tx_t *tx) { return !tx->next_ready && !tx->preparing; }

/* Single-channel convenience: next chip of channel 0. */
uint8_t rs_tx_next_chip(rs_tx_t *tx);
/* For bindings and tests: rs_encode_packet as a linkable function, and the structure's size (a
 * binding that mirrors rs_tx_t checks its layout against it). */
int rs_tx_encode(uint8_t id, uint16_t seed, uint8_t payload, uint8_t *chips);
size_t rs_tx_sizeof(void);

#ifdef __cplusplus
}
#endif
#endif
