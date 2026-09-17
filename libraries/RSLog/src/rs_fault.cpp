/*
 * Hard fault capture for Cortex-M4 (RA4M1). Overrides the weak
 * HardFault_Handler from the FSP startup code. Captures the stacked PC/LR
 * and the fault status registers, then hands over to RSLog which blinks the
 * fault record forever (no interrupts needed).
 */
#include "RSLog.h"

static void hex(char *dst, uint32_t v, int digits)
{
    static const char *h = "0123456789abcdef";
    for (int i = digits - 1; i >= 0; i--) { dst[i] = h[v & 0xF]; v >>= 4; }
}

extern "C" __attribute__((used)) void rs_hardfault_c(uint32_t *frame)
{
    uint32_t pc = frame[6], lr = frame[5];
    uint32_t cfsr = SCB->CFSR;
    /* "HF p=XXXXXXXX l=XXXXXXXX c=XXXX" = 31 chars (c = low 16 bits of CFSR: MMFSR|BFSR) */
    char t[RS_MSG_MAX_LEN + 1];
    const char *tpl = "HF p=........ l=........ c=....";
    for (int i = 0; i < RS_MSG_MAX_LEN; i++) t[i] = tpl[i];
    t[RS_MSG_MAX_LEN] = 0;
    hex(t + 5, pc, 8);
    hex(t + 16, lr, 8);
    hex(t + 27, cfsr & 0xFFFF, 4);
    /* mini backtrace: return addresses on the stack above the exception frame (odd Thumb
     * addresses inside the 256 KB code flash), logged as ERROR and copied into the death loop */
    {
        char b[RS_MSG_MAX_LEN + 1] = "bt"; int n = 2, nb = 0;
        for (int i = 8; i < 8 + 96 && nb < 3; i++) {
            uint32_t w = frame[i];
            if ((w & 1) && w >= 0x100 && w < 0x40000 && w != lr && w != (pc | 1)) {
                b[n++] = ' '; hex(b + n, w, 5); n += 5; b[n] = 0; nb++;
            }
        }
        if (nb) RSLog.log(RS_LVL_ERROR, "%s", b);
    }
    RSLogClass::_persistAndLoop(t);
}

extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "b rs_hardfault_c  \n");
}
