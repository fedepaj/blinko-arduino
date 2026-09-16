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
