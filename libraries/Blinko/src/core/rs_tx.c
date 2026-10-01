#include "rs_tx.h"
#include "rs_pack.h"

static void rs_memcpy(uint8_t *d, const uint8_t *s, size_t n) { while (n--) *d++ = *s++; }

void rs_tx_init(rs_tx_t *tx)
{
    uint8_t *p = (uint8_t *)tx;
    for (size_t i = 0; i < sizeof(*tx); i++) p[i] = 0;
    for (int c = 0; c < RS_MAX_CHANNELS; c++) tx->chip_pos[c] = RS_PKT_CHIPS;   /* fetch on first chip */
    tx->repeat = 1;
    tx->nchan = 1;
    tx->fault_weight = 1;
}

void rs_tx_set_fault_weight(rs_tx_t *tx, uint8_t w)
{
    if (w < 1) w = 1;
    if (w > RS_TX_MAX_FAULT_WEIGHT) w = RS_TX_MAX_FAULT_WEIGHT;
    tx->fault_weight = w;
}

void rs_tx_set_channels(rs_tx_t *tx, uint8_t nchan, uint32_t pilot_period)
{
    tx->nchan = (nchan == 3) ? 3 : 1;
    tx->pilot_period = (tx->nchan == 3) ? pilot_period : 0;
    tx->pilot_pos = 0; tx->in_pilot = 0;
}

static void slot_fill(rs_slot_t *s, uint8_t level, const char *text, size_t len, uint32_t seq)
{
    uint8_t packed[RS_MSG_MAX_LEN];
    size_t plen = rs_pack6(text, len, packed, sizeof(packed));
    if (plen > 0) {
        rs_memcpy(s->data, packed, plen);
        s->len = (uint8_t)plen; s->packed = 1;
    } else {
        if (len > RS_MSG_MAX_LEN) len = RS_MSG_MAX_LEN;
        if (len == 0) { s->valid = 0; return; }
        rs_memcpy(s->data, (const uint8_t *)text, len);
        s->len = (uint8_t)len; s->packed = 0;
    }
    s->level = (uint8_t)(level & 7u);
    s->seq = seq;
    s->next_seed = 0;
    s->valid = 1;
}

uint8_t rs_tx_log(rs_tx_t *tx, uint8_t level, const char *text, size_t len)
{
    uint8_t id = tx->next_log_slot;
    tx->next_log_slot = (uint8_t)((id + 1) % RS_NUM_LOG_SLOTS);
    slot_fill(&tx->slots[id], level, text, len, ++tx->seq_counter);
    return id;
}

void rs_tx_set_slot(rs_tx_t *tx, uint8_t id, uint8_t level, const char *text, size_t len)
{
    if (id >= RS_NUM_SLOTS) return;
    slot_fill(&tx->slots[id], level, text, len, ++tx->seq_counter);
}

void rs_tx_clear_slot(rs_tx_t *tx, uint8_t id)
{
    if (id < RS_NUM_SLOTS) tx->slots[id].valid = 0;
}

void rs_tx_set_burst(rs_tx_t *tx, uint32_t on_chips, uint32_t off_chips)
{
    tx->burst_on = on_chips; tx->burst_off = off_chips; tx->burst_pos = 0; tx->in_pause = 0;
}

/* FAULT, STATUS, then log slots newest first. With fault_weight w > 1 every other visit is
 * followed by w-1 extra FAULT visits: [F S F F L1 F F L2 F F ...] for w = 3. */
static void build_round(rs_tx_t *tx)
{
    uint8_t n = 0;
    int fault = tx->slots[RS_SLOT_FAULT].valid;
    uint8_t extra = fault ? (uint8_t)(tx->fault_weight - 1) : 0;
    if (fault) tx->round[n++] = RS_SLOT_FAULT;
    if (tx->slots[RS_SLOT_STATUS].valid) {
        tx->round[n++] = RS_SLOT_STATUS;
        for (uint8_t k = 0; k < extra; k++) tx->round[n++] = RS_SLOT_FAULT;
    }
    uint8_t used = 0;
    for (;;) {
        int best = -1; uint32_t best_seq = 0;
        for (uint8_t i = 0; i < RS_NUM_LOG_SLOTS; i++) {
            if (!tx->slots[i].valid || (used & (1u << i))) continue;
            if (best < 0 || tx->slots[i].seq > best_seq) { best = i; best_seq = tx->slots[i].seq; }
        }
        if (best < 0) break;
        used |= (uint8_t)(1u << best);
        tx->round[n++] = (uint8_t)best;
        for (uint8_t k = 0; k < extra; k++) tx->round[n++] = RS_SLOT_FAULT;
    }
    tx->round_len = n;
    tx->round_pos = 0;
}

static uint8_t coded_payload(const rs_slot_t *s, uint16_t seed)
{
    if (seed < s->len) return s->data[seed];                 /* systematic */
    uint32_t m = rs_code_mask(seed, s->len);
    uint8_t v = 0;
    for (uint8_t i = 0; i < s->len; i++) if (m & (1u << i)) v ^= s->data[i];
    return v;
}

/* Visit layout: [META C1 C2] d.. [META C1 C2] d.. (control triplet at start and
 * mid-visit so any camera window of ~len/2 packets contains one). */
static void control_packet(rs_tx_t *tx, uint8_t k, uint8_t *id, uint16_t *seed, uint8_t *payload)
{
    *id = tx->cur_id;
    if (k == 0) { *seed = tx->cur.packed ? RS_SEED_META_PACKED : RS_SEED_META; *payload = rs_meta(tx->cur.len, tx->cur.level); }
    else if (k == 1) { *seed = RS_SEED_MSGCRC;  *payload = rs_crc8(tx->cur.data, tx->cur.len); }
    else             { *seed = RS_SEED_MSGCRC2; *payload = rs_crc8b(tx->cur.data, tx->cur.len); }
}

void rs_tx_next_packet(rs_tx_t *tx, uint8_t *id, uint16_t *seed, uint8_t *payload)
{
    if (tx->cur_sent == 0) {
        /* start of a visit: pick the next slot in the round */
        if (tx->round_pos >= tx->round_len) build_round(tx);
        if (tx->round_len == 0) {
            *id = RS_SLOT_STATUS; *seed = RS_SEED_META; *payload = rs_meta(0, RS_LVL_STATUS);
            tx->packets_sent++;
            return;
        }
        tx->cur_id = tx->round[tx->round_pos++];
        tx->cur = tx->slots[tx->cur_id];          /* snapshot */
        if (!tx->cur.valid) { rs_tx_next_packet(tx, id, seed, payload); return; }
        tx->visit_len = (uint8_t)(tx->cur.len + 6);
    }
    uint8_t pos = tx->cur_sent;
    uint8_t half = (uint8_t)(tx->cur.len / 2);
    if (pos < 3) {
        control_packet(tx, pos, id, seed, payload);
    } else if (pos >= 3 + half && pos < 6 + half) {
        control_packet(tx, (uint8_t)(pos - 3 - half), id, seed, payload);
    } else {
        rs_slot_t *live = &tx->slots[tx->cur_id];
        uint16_t s = tx->cur.next_seed;
        *id = tx->cur_id;
        *seed = s;
        *payload = coded_payload(&tx->cur, s);
        s = (uint16_t)((s + 1) > RS_SEED_CODED_MAX ? tx->cur.len : s + 1);  /* wrap into coded range */
        tx->cur.next_seed = s;
        if (live->valid && live->seq == tx->cur.seq) live->next_seed = s;  /* persist unless replaced */
    }
    tx->cur_sent++;
    if (tx->cur_sent >= tx->visit_len) tx->cur_sent = 0;
    tx->packets_sent++;
}

static void fetch_packet(rs_tx_t *tx, int c)
{
    if (tx->rep_left[c] > 0) { tx->rep_left[c]--; tx->chip_pos[c] = 0; return; }   /* another copy of the same packet */
    uint8_t id, payload; uint16_t seed;
    rs_tx_next_packet(tx, &id, &seed, &payload);
    rs_encode_packet(id, seed, payload, tx->chips[c]);
    tx->chip_pos[c] = 0;
    tx->rep_left[c] = (uint8_t)(tx->repeat > 1 ? tx->repeat - 1 : 0);
}

void rs_tx_set_repeat(rs_tx_t *tx, uint8_t n)
{
    tx->repeat = n < 1 ? 1 : (n > 4 ? 4 : n);
}

void rs_tx_next_chips(rs_tx_t *tx, uint8_t out[RS_MAX_CHANNELS])
{
    out[0] = out[1] = out[2] = 0;
    if (tx->in_pilot) {
        uint32_t k = tx->pilot_idx++ / RS_PILOT_P;      /* 0..8 */
        if (k == 2) out[0] = 1; else if (k == 4) out[1] = 1; else if (k == 6) out[2] = 1;
        if (tx->pilot_idx >= RS_PILOT_CHIPS) { tx->in_pilot = 0; tx->pilot_pos = 0; }
        return;
    }
    if (tx->in_pause) {
        if (++tx->burst_pos >= tx->burst_off) { tx->in_pause = 0; tx->burst_pos = 0; }
        return;
    }
    /* all channels advance in lockstep, so a boundary on channel 0 is a boundary on all */
    if (tx->chip_pos[0] >= RS_PKT_CHIPS) {
        if (tx->pilot_period && tx->pilot_pos >= tx->pilot_period) {
            tx->in_pilot = 1; tx->pilot_idx = 0;
            rs_tx_next_chips(tx, out);
            return;
        }
        if (tx->burst_on && tx->burst_pos >= tx->burst_on) {
            tx->in_pause = 1; tx->burst_pos = 0;
            return;
        }
        for (int c = 0; c < tx->nchan; c++) fetch_packet(tx, c);
    }
    if (tx->burst_on) tx->burst_pos++;
    if (tx->pilot_period) tx->pilot_pos++;
    for (int c = 0; c < tx->nchan; c++) out[c] = tx->chips[c][tx->chip_pos[c]++];
    if (tx->nchan == 1) { out[1] = out[0]; out[2] = out[0]; }
}

uint8_t rs_tx_next_chip(rs_tx_t *tx)
{
    uint8_t out[RS_MAX_CHANNELS];
    rs_tx_next_chips(tx, out);
    return out[0];
}

int rs_tx_encode(uint8_t id, uint16_t seed, uint8_t payload, uint8_t *chips) { return rs_encode_packet(id, seed, payload, chips); }
