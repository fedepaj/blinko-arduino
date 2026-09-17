/*
 * rs_pack.h — lossless 6-bit text packing for log messages.
 * Symbols: 0 space, 1-26 a-z, 27-36 0-9, 37 SHIFT (next symbol is uppercase
 * a-z), 38-62 punctuation "=:.,-_/#%()[]<>+*@!?';&|$", 63 ESC (next 8 bits =
 * raw byte). Typical log text shrinks by 20-25 %.
 */
#ifndef RS_PACK_H
#define RS_PACK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pack text into out (capacity out_cap bytes). Returns packed length, or 0 if
 * it does not fit or would not be shorter than the input. */
size_t rs_pack6(const char *text, size_t len, uint8_t *out, size_t out_cap);

/* Unpack into text (capacity text_cap, NUL-terminated). Returns text length. */
size_t rs_unpack6(const uint8_t *in, size_t len, char *text, size_t text_cap);

#ifdef __cplusplus
}
#endif
#endif
