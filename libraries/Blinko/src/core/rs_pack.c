#include "rs_pack.h"

static const char PUNCT[] = "=:.,-_/#%()[]<>+*@!?';&|$";   /* 25 symbols: 38..62 */
#define SYM_SHIFT 37
#define SYM_ESC   63

static int sym_of(char c, int *shift)
{
    *shift = 0;
    if (c == ' ') return 0;
    if (c >= 'a' && c <= 'z') return 1 + (c - 'a');
    if (c >= 'A' && c <= 'Z') { *shift = 1; return 1 + (c - 'A'); }
    if (c >= '0' && c <= '9') return 27 + (c - '0');
    for (int i = 0; PUNCT[i]; i++) if (PUNCT[i] == c) return 38 + i;
    return -1;
}

typedef struct { uint8_t *out; size_t cap; size_t nbits; } bw_t;
static int put_bits(bw_t *w, uint32_t v, int n)
{
    for (int i = n - 1; i >= 0; i--) {
        size_t byte = w->nbits >> 3;
        if (byte >= w->cap) return 0;
        if ((w->nbits & 7) == 0) w->out[byte] = 0;
        if ((v >> i) & 1u) w->out[byte] |= (uint8_t)(0x80u >> (w->nbits & 7));
        w->nbits++;
    }
    return 1;
}

size_t rs_pack6(const char *text, size_t len, uint8_t *out, size_t out_cap)
{
    bw_t w = { out, out_cap, 0 };
    for (size_t i = 0; i < len; i++) {
        int shift, s = sym_of(text[i], &shift);
        if (s < 0) {
            if (!put_bits(&w, SYM_ESC, 6) || !put_bits(&w, (uint8_t)text[i], 8)) return 0;
        } else {
            if (shift && !put_bits(&w, SYM_SHIFT, 6)) return 0;
            if (!put_bits(&w, (uint32_t)s, 6)) return 0;
        }
    }
    size_t n = (w.nbits + 7) >> 3;
    /* pad with 1s (an incomplete ESC-less symbol of all ones = ESC marker without byte -> ignored) */
    while (w.nbits & 7) { put_bits(&w, 1, 1); }
    return (n < len) ? n : 0;
}

size_t rs_pack6_fit(const char *text, size_t len, uint8_t *out, size_t out_cap, size_t *consumed)
{
    bw_t w = { out, out_cap, 0 };
    size_t i = 0;
    for (; i < len; i++) {
        int shift, s = sym_of(text[i], &shift);
        size_t need = s < 0 ? 14u : (shift ? 12u : 6u);          /* a character goes in whole or not at all */
        if (w.nbits + need > out_cap * 8u) break;
        if (s < 0) { put_bits(&w, SYM_ESC, 6); put_bits(&w, (uint8_t)text[i], 8); }
        else { if (shift) put_bits(&w, SYM_SHIFT, 6); put_bits(&w, (uint32_t)s, 6); }
    }
    size_t n = (w.nbits + 7) >> 3;
    while (w.nbits & 7) put_bits(&w, 1, 1);                       /* pad with 1s, as rs_pack6 */
    *consumed = i;
    return n;
}

size_t rs_unpack6(const uint8_t *in, size_t len, char *text, size_t text_cap)
{
    size_t nbits = len * 8, pos = 0, n = 0;
    int shift = 0;
    while (pos + 6 <= nbits && n + 1 < text_cap) {
        uint32_t s = 0;
        for (int i = 0; i < 6; i++, pos++) s = (s << 1) | ((in[pos >> 3] >> (7 - (pos & 7))) & 1u);
        char c;
        if (s == SYM_ESC) {
            if (pos + 8 > nbits) break;           /* padding */
            uint32_t b = 0;
            for (int i = 0; i < 8; i++, pos++) b = (b << 1) | ((in[pos >> 3] >> (7 - (pos & 7))) & 1u);
            c = (char)b;
        } else if (s == SYM_SHIFT) { shift = 1; continue; }
        else if (s == 0) c = ' ';
        else if (s <= 26) c = (char)((shift ? 'A' : 'a') + (s - 1));
        else if (s <= 36) c = (char)('0' + (s - 27));
        else c = PUNCT[s - 38];
        shift = 0;
        text[n++] = c;
    }
    text[n] = 0;
    return n;
}
