#include "Blinko.h"
#include <FspTimer.h>
#include <pwm.h>
#include <DataFlashBlockDevice.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BLINKO_FAULT_MAGIC 0x52534641u   /* "RSFA" */
#define BLINKO_BOOT_MAGIC  0x52534254u   /* "RSBT" */
#define BLINKO_EE_MAGIC    0x52534545u   /* "RSEE" */
#define BLINKO_DF_OFFSET   (8192 - 1024)   /* last 1 KB data-flash block (raw, no virtual-EEPROM layer) */
#define BLINKO_LOAD_MAGIC  0x4C4F4144u   /* "LOAD": set while reading flash at boot (boot-loop guard) */

BlinkoClass Blinko;

/* Reset-surviving record. The .noinit section of the Arduino linker script sits
 * in low RAM, which the Nano R4 bootloader zeroes on every reset (verified on
 * hardware); high RAM survives. We use a fixed address just below the main
 * stack region (__StackLimit = 0x20007B00 on the R4 linker script). The heap
 * only reaches it after ~25 KB of allocations. */
#ifndef BLINKO_RECORD_ADDR
#define BLINKO_RECORD_ADDR 0x20007A00u
#endif
#define blinko_fault_record (*(rs_fault_record_t *)BLINKO_RECORD_ADDR)

static FspTimer s_timer;
static uint8_t s_timer_type = 255, s_timer_ch = 255, s_timer_step = 0;   /* for diagnostics */
static volatile uint32_t s_ticks = 0;
static PwmOut *s_pwm[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH];   /* one per LED pin while brightness < 100 */
#define BLINKO_PWM_HZ 240000.0f                              /* carrier well above a camera exposure (a 15 us row sees 3-4 periods); 1 MHz left the GPT too few counts and the LED dark */

struct rs_ee_record_t { uint32_t magic; uint32_t boot_count; char text[RS_MSG_MAX_LEN + 1]; };

/* Raw data-flash persistence. The FSP flash_lp driver is blocking (no BGO, no
 * IRQ), so these work from fatal() and from the hard fault handler. */
/* Note: the core's DataFlashBlockDevice::init() fails when already open and
 * read() reports an error code even on success, so we call erase/program/read
 * directly (they open the driver themselves) and validate by magic. */
static bool df_write(const rs_ee_record_t &r)
{
    DataFlashBlockDevice &bd = DataFlashBlockDevice::getInstance();
    if (bd.erase(BLINKO_DF_OFFSET, 1024) != 0) return false;
    return bd.program(&r, BLINKO_DF_OFFSET, sizeof(r)) == 0;
}
static bool df_read(rs_ee_record_t &r)
{
    DataFlashBlockDevice &bd = DataFlashBlockDevice::getInstance();
    (void)bd.read(&r, BLINKO_DF_OFFSET, sizeof(r));
    return true;
}

/* Identifies the firmware build: a different id at boot means a new upload,
 * whose bootloader-driven reset must not be reported as a watchdog event. */
static uint32_t fw_build_id()
{
    const char *s = __DATE__ " " __TIME__;
    uint32_t h = 2166136261u;
    while (*s) { h ^= (uint8_t)*s++; h *= 16777619u; }
    return h;
}

static void timer_cb(timer_callback_args_t *) { Blinko._tick(); }

/* The next packets are chosen and encoded here, in the lowest-priority exception, which the chip
 * interrupt preempts: three packets take about 120 us on this MCU, six chip periods at T = 60 us,
 * and done inside the chip interrupt they cost 8 % of its ticks (every packet 89 chips long
 * instead of 82, measured with the tick counter). The chip interrupt asks for it with a pended
 * PendSV as soon as the previous packets went on air. */
extern "C" void PendSV_Handler(void) { rs_tx_prepare(&Blinko.tx()); }

/* Masks the interrupts for a scope and puts back what was there before: a Blinko call made
 * inside the sketch's own noInterrupts() block must not switch them back on (interrupts() does). */
struct IrqLock {
    uint32_t primask;
    IrqLock() : primask(__get_PRIMASK()) { __disable_irq(); }
    ~IrqLock() { __set_PRIMASK(primask); }
};

/* Write the fault text to data flash unless it is there already. The death loop writes the
 * record and the next boot, finding it in RAM too, used to write it again: two erase cycles per
 * fault, which a crash loop behind a watchdog repeats every few seconds. */
static void persist_fault(const char *text, uint32_t boot_count);

/* ---------------------------------------------------------------- pins */

/* Dimmed output. The LED pin runs a hardware PWM at the brightness duty and the chip only
 * decides what the pin is connected to: the timer output (lit) or a plain GPIO at the off level
 * (dark). That is one write of the pin's function register per chip, safe inside the chip
 * interrupt; calling the PWM driver from there (duty changes) hung the MCU. */
static volatile uint32_t *s_pfs_reg[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH];
static uint32_t s_pfs_on[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH], s_pfs_off[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH];
static inline void pfs_write(volatile uint32_t *reg, uint32_t v)
{
    R_BSP_PinAccessEnable();                            /* the BSP's unlock of the pin function registers (counted, interrupt-safe) */
    *reg = v;
    R_BSP_PinAccessDisable();
}

/* One store to the port's set/reset register (PCNTR3: bits 0-15 set, 16-31 reset). digitalWrite()
 * and R_BSP_PinWrite() go through the pin function register and its write protection; four
 * digitalWrite() per chip took most of a 15 us chip period. */
static inline void pin_write(uint8_t pin, bool high)
{
    bsp_io_port_pin_t pp = g_pin_cfg[pin].pin;
    R_PORT0_Type *port = (R_PORT0_Type *)((uintptr_t)R_PORT0 + (pp >> 8) * ((uintptr_t)R_PORT1 - (uintptr_t)R_PORT0));
    port->PCNTR3 = high ? (1u << (pp & 0xFF)) : (1u << (pp & 0xFF)) << 16;
}

/* Diagnostics: the pin function register of the red LED as configured for lit / dark and now. */
const char *BlinkoClass::pfsInfo()
{
    static char buf[80];
    volatile uint32_t *reg = s_pfs_reg[0][0];
    snprintf(buf, sizeof(buf), "pfs on %08lx off %08lx now %08lx", (unsigned long)s_pfs_on[0][0], (unsigned long)s_pfs_off[0][0], reg ? (unsigned long)*reg : 0ul);
    return buf;
}

/* What the chip interrupt writes for each LED pin, worked out once in begin(): the port's
 * set/reset register and the word that lights the LED and the one that darkens it. */
static R_PORT0_Type *s_led_port[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH];
static uint32_t s_led_on[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH], s_led_off[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH];
static void cache_led_pins(const BlinkoConfig &cfg)
{
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) {
        uint8_t pin = cfg.ch_pins[c][k];
        if (pin == BLINKO_NO_PIN) { s_led_port[c][k] = nullptr; continue; }
        bsp_io_port_pin_t pp = g_pin_cfg[pin].pin;
        uint32_t set = 1u << (pp & 0xFF), reset = set << 16;
        s_led_port[c][k] = (R_PORT0_Type *)((uintptr_t)R_PORT0 + (pp >> 8) * ((uintptr_t)R_PORT1 - (uintptr_t)R_PORT0));
        s_led_on[c][k] = cfg.ch_active_low[c][k] ? reset : set;
        s_led_off[c][k] = cfg.ch_active_low[c][k] ? set : reset;
    }
}

void BlinkoClass::_writeChips(const uint8_t chips[RS_MAX_CHANNELS])
{
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) {
        for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) {
            if (!s_led_port[c][k]) continue;
            if (s_pfs_reg[c][k]) pfs_write(s_pfs_reg[c][k], chips[c] ? s_pfs_on[c][k] : s_pfs_off[c][k]);
            else s_led_port[c][k]->PCNTR3 = chips[c] ? s_led_on[c][k] : s_led_off[c][k];
        }
    }
}

/* The second output (A or B) of a GPT channel that one of our PwmOut objects already runs.
 * Two LED pins can sit on the two outputs of one timer (the Nano R4's green and blue: GPT6 B and
 * A), and a second PwmOut on a channel in use goes through a path of the core that only works
 * for analogWrite() pins: it left GPT6 misconfigured and green and blue dark at any brightness
 * under 100. So the second pin gets no PwmOut: its output is enabled on the first pin's timer,
 * with the same waveform, and its duty set there. */
static bool pwm_second_output(PwmOut *owner, uint8_t pin, uint8_t gpt_channel, bool on_a, float duty_perc)
{
    R_GPT0_Type *gpt = (R_GPT0_Type *)((uintptr_t)R_GPT0 + gpt_channel * ((uintptr_t)R_GPT1 - (uintptr_t)R_GPT0));
    R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[pin].pin, (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
    uint32_t gtior = gpt->GTIOR;
    if (on_a) {      /* the waveform bits of the output already running, copied to the other one, and its enable */
        uint32_t fn = (gtior & R_GPT0_GTIOR_GTIOB_Msk) >> R_GPT0_GTIOR_GTIOB_Pos;
        gtior = (gtior & ~R_GPT0_GTIOR_GTIOA_Msk) | (fn << R_GPT0_GTIOR_GTIOA_Pos) | R_GPT0_GTIOR_OAE_Msk;
    } else {
        uint32_t fn = (gtior & R_GPT0_GTIOR_GTIOA_Msk) >> R_GPT0_GTIOR_GTIOA_Pos;
        gtior = (gtior & ~R_GPT0_GTIOR_GTIOB_Msk) | (fn << R_GPT0_GTIOR_GTIOB_Pos) | R_GPT0_GTIOR_OBE_Msk;
    }
    gpt->GTWP = 0xA500u;                                  /* the timer's registers are write-protected */
    gpt->GTIOR = gtior;
    gpt->GTWP = 0xA501u;
    uint32_t counts = (uint32_t)((float)owner->get_timer()->get_period_raw() * duty_perc / 100.0f);
    return owner->get_timer()->set_duty_cycle(counts, on_a ? CHANNEL_A : CHANNEL_B);
}

/* Brightness below 100 % puts every LED pin on a hardware PWM (the Nano R4's LED pins sit on
 * GPT channels: red 5A, green 6B, blue 6A, builtin 4B) at a fixed duty; the chips then switch
 * the pin between that output and a GPIO at the off level (see _writeChips). */
void BlinkoClass::_applyBrightness()
{
    PwmOut *owner_of[8] = { nullptr };                    /* the PwmOut running each GPT channel */
    if (_cfg.brightness < 100)
        for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) if (s_pwm[c][k]) {
            auto cfgs = getPinCfgs(_cfg.ch_pins[c][k], PIN_CFG_REQ_PWM);
            if (cfgs[0] && GET_CHANNEL(cfgs[0]) < 8) owner_of[GET_CHANNEL(cfgs[0])] = s_pwm[c][k];
        }
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) {
        for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) {
            uint8_t pin = _cfg.ch_pins[c][k];
            if (pin == BLINKO_NO_PIN) continue;
            float duty = _cfg.ch_active_low[c][k] ? 100.0f - (float)_cfg.brightness : (float)_cfg.brightness;
            if (_cfg.brightness >= 100) {
                s_pfs_reg[c][k] = nullptr;
                if (s_pwm[c][k]) { s_pwm[c][k]->end(); delete s_pwm[c][k]; s_pwm[c][k] = nullptr; }
                pinMode(pin, OUTPUT); digitalWrite(pin, _cfg.ch_active_low[c][k] ? HIGH : LOW);
                continue;
            }
            auto cfgs = getPinCfgs(pin, PIN_CFG_REQ_PWM);
            if (!cfgs[0] || IS_PIN_AGT_PWM(cfgs[0]) || GET_CHANNEL(cfgs[0]) >= 8) { s_pfs_reg[c][k] = nullptr; continue; }   /* no GPT output on this pin: it stays at full brightness */
            uint8_t gch = GET_CHANNEL(cfgs[0]);
            if (s_pwm[c][k]) {
                s_pwm[c][k]->pulse_perc(duty);
            } else if (owner_of[gch]) {
                if (!pwm_second_output(owner_of[gch], pin, gch, IS_PWM_ON_A(cfgs[0]), duty)) { pinMode(pin, OUTPUT); s_pfs_reg[c][k] = nullptr; continue; }
            } else {
                s_pwm[c][k] = new PwmOut(pin);
                if (!s_pwm[c][k]->begin(BLINKO_PWM_HZ, duty)) { delete s_pwm[c][k]; s_pwm[c][k] = nullptr; s_pfs_reg[c][k] = nullptr; continue; }
                owner_of[gch] = s_pwm[c][k];
            }
            bsp_io_port_pin_t pp = g_pin_cfg[pin].pin;
            volatile uint32_t *reg = &R_PFS->PORT[pp >> 8].PIN[pp & 0xFF].PmnPFS;
            s_pfs_on[c][k] = *reg;                                       /* the function PwmOut configured (PMR set, PSEL = GPT) */
            /* dark: the same register with PMR cleared (GPIO instead of the timer output) and the
             * GPIO driven at the off level; PSEL stays, since it must not change while PMR is set */
            s_pfs_off[c][k] = (s_pfs_on[c][k] & ~(1u << 16)) | (1u << 2) | (_cfg.ch_active_low[c][k] ? 1u : 0u);
            s_pfs_reg[c][k] = reg;
        }
    }
}

const char *BlinkoClass::timerInfo()
{
    static char buf[96];
    snprintf(buf, sizeof(buf), "timer %s ch %d step %d running %d ticks %lu pwm %d", s_timer_type == AGT_TIMER ? "AGT" : s_timer_type == GPT_TIMER ? "GPT" : "none", (int)s_timer_ch, (int)s_timer_step, (int)_running, (unsigned long)s_ticks, (int)(s_pwm[0][0] != nullptr));
    return buf;
}

void BlinkoClass::setBrightness(uint8_t percent)
{
    _cfg.brightness = percent < 1 ? 1 : (percent > 100 ? 100 : percent);
    /* the PWM channels are claimed with the chip timer stopped, then the chip timer restarts on
     * a channel that is still free: PwmOut::begin re-initializes its GPT channel, and the core's
     * timer allocation does not know which channels the LED pins will take */
    bool was_running = _running;
    if (was_running) { s_timer.stop(); s_timer.end(); _running = false; }
    _applyBrightness();
    _strobe = false;                                    /* the timer restarts at the chip rate: a strobe in progress ends */
    if (was_running) _running = _startTimer(1.0e6f / (float)cellMicros());
}

void BlinkoClass::_writeAll(uint8_t level)
{
    uint8_t v[RS_MAX_CHANNELS] = { level, level, level };
    _writeChips(v);
}

void BlinkoClass::_tick()
{
    s_ticks++;
    if (!_enabled) return;
    if (_strobe) { _strobe_level ^= 1; _writeAll(_strobe_level); return; }
    /* The pins first, with the chips worked out in the previous tick, then the next ones: the
     * time from the timer's overflow to the pin write is then constant. Working them out first
     * put the encoder's time before the write, and at a packet boundary (three packets to encode)
     * that is several chip periods: the last chip of every packet was stretched and the gap
     * before the sync eaten. The death loop below does the same. */
    _writeChips(_next_chips);
    rs_tx_next_chips(&_tx, _next_chips);
    if (rs_tx_wants_prepare(&_tx)) SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

/* --------------------------------------------------------------- timer */

bool BlinkoClass::_startTimer(float hz)
{
    /* The LED pins' PWM (brightness) lives on GPT channels (Nano R4: red 5A, green 6B, blue 6A,
     * builtin 4B) and PwmOut::begin re-initializes its channel, which would silence a chip timer
     * there: those channels are reserved before the chip timer picks one. (An AGT would not
     * collide, but the AGT overflow interrupt did not fire through FspTimer on this core.) */
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++)
        for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) {
            uint8_t pin = _cfg.ch_pins[c][k];
            if (pin == BLINKO_NO_PIN) continue;
            auto cfgs = getPinCfgs(pin, PIN_CFG_REQ_PWM);
            if (cfgs[0]) FspTimer::set_initial_timer_channel_as_pwm(GPT_TIMER, GET_CHANNEL(cfgs[0]));
        }
    uint8_t type = GPT_TIMER;
    int8_t ch = FspTimer::get_available_timer(type);
    if (ch < 0) ch = FspTimer::get_available_timer(type, true);
    s_timer_type = type; s_timer_ch = (uint8_t)ch; s_timer_step = 0;
    if (ch < 0) return false;
    s_timer_step = 1; if (!s_timer.begin(TIMER_MODE_PERIODIC, type, (uint8_t)ch, hz, 50.0f, timer_cb, nullptr)) return false;
    s_timer_step = 2; if (!s_timer.setup_overflow_irq()) return false;
    s_timer_step = 3; if (!s_timer.open()) return false;
    s_timer_step = 4; return s_timer.start();
}

/* --------------------------------------------------------------- begin */

bool BlinkoClass::begin(const BlinkoConfig &cfg)
{
    if (_running) end();
    _cfg = cfg;
    /* a configuration that would divide by zero or leave the LEDs dark is brought into range */
    if (_cfg.chip_us < BLINKO_MIN_CHIP_US) _cfg.chip_us = BLINKO_MIN_CHIP_US;
    if (_cfg.fault_chip_us == 0) _cfg.fault_chip_us = 120;
    if (_cfg.fault_chip_us < BLINKO_MIN_CHIP_US) _cfg.fault_chip_us = BLINKO_MIN_CHIP_US;
    if (_cfg.fault_repeat == 0) _cfg.fault_repeat = 3;
    if (_cfg.brightness < 1 || _cfg.brightness > 100) _cfg.brightness = 100;
    _cfg.channels = (_cfg.channels == 3) ? 3 : 1;
    _strobe = false; _enabled = true;
    _next_chips[0] = _next_chips[1] = _next_chips[2] = 0;
    cache_led_pins(_cfg);
    rs_tx_init(&_tx);
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++)
        for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++)
            if (_cfg.ch_pins[c][k] != BLINKO_NO_PIN) pinMode(_cfg.ch_pins[c][k], OUTPUT);
    if (_cfg.brightness < 100) _applyBrightness();      /* before the chip timer: its channel must stay clear of the LED pins' */
    pinMode(_cfg.fault_pin, OUTPUT);
    _writeAll(0);

    _readResetCause();

    /* Warm reset bookkeeping */
    rs_fault_record_t &fr = blinko_fault_record;
    bool warm = (fr.boot_magic == BLINKO_BOOT_MAGIC);
    if (!warm) { fr.boot_magic = BLINKO_BOOT_MAGIC; fr.boot_count = 0; fr.checkpoint[0] = 0; fr.magic = 0; fr.loading = 0; }
    if (fr.fw_id != fw_build_id()) { fr.fw_id = fw_build_id(); fr.checkpoint[0] = 0; warm = false; /* fresh upload */ }
    fr.boot_count++;

    if (fr.magic == BLINKO_FAULT_MAGIC) {
        /* a fault was recorded before the reset: persist it and announce */
        fr.text[RS_MSG_MAX_LEN] = 0;
        strncpy(_fault_text, fr.text, RS_MSG_MAX_LEN); _fault_text[RS_MSG_MAX_LEN] = 0;
        fr.magic = 0;
        if (_cfg.persist_faults) persist_fault(_fault_text, fr.boot_count);
    } else if (warm && strstr(_reset_cause, "WDT")) {     /* WDT or IWDT */
        /* watchdog reset with no explicit record: report last checkpoint */
        fr.checkpoint[BLINKO_CHECKPOINT_LEN - 1] = 0;
        snprintf(_fault_text, sizeof(_fault_text), "WDT reset @%s", fr.checkpoint[0] ? fr.checkpoint : "?");
        if (_cfg.persist_faults) persist_fault(_fault_text, fr.boot_count);
    } else if (_cfg.persist_faults) {
        if (warm && fr.loading == BLINKO_LOAD_MAGIC) {
            /* the previous boot crashed while reading the record: don't retry, wipe it */
            rs_ee_record_t ee = { 0, 0, { 0 } };
            df_write(ee);
            strncpy(_fault_text, "boot-loop guard: record wiped", RS_MSG_MAX_LEN);
        } else {
            fr.loading = BLINKO_LOAD_MAGIC;
            _loadPersistedFault();
        }
    }
    fr.loading = 0;
    fr.checkpoint[0] = 0;

    if (_fault_text[0]) _setSlot(RS_SLOT_FAULT, RS_LVL_FAULT, _fault_text, strlen(_fault_text));

    if (_cfg.announce_boot) status("boot#%lu rst=%s id=%04x", (unsigned long)fr.boot_count, _reset_cause, boardId());

    _applyTiming();
    rs_tx_set_repeat(&_tx, _cfg.repeat);
    NVIC_SetPriority(PendSV_IRQn, (1u << __NVIC_PRIO_BITS) - 1u);   /* below every interrupt, the chip timer's first of all */
    _running = _startTimer(1.0e6f / (float)cellMicros());
    return _running;
}

void BlinkoClass::end()
{
    if (_running) { s_timer.stop(); s_timer.end(); _running = false; }
    _writeAll(0);
}

static void persist_fault(const char *text, uint32_t boot_count)
{
    rs_ee_record_t ee;
    df_read(ee);
    if (ee.magic == BLINKO_EE_MAGIC && strncmp(ee.text, text, RS_MSG_MAX_LEN) == 0) return;
    rs_ee_record_t w = { BLINKO_EE_MAGIC, boot_count, { 0 } };
    strncpy(w.text, text, RS_MSG_MAX_LEN);
    df_write(w);
}

void BlinkoClass::_loadPersistedFault()
{
    rs_ee_record_t ee;
    df_read(ee);
    if (ee.magic == BLINKO_EE_MAGIC) {
        ee.text[RS_MSG_MAX_LEN] = 0;
        strncpy(_fault_text, ee.text, RS_MSG_MAX_LEN); _fault_text[RS_MSG_MAX_LEN] = 0;
    }
}

void BlinkoClass::clearFault()
{
    _fault_text[0] = 0;
    { IrqLock lock; rs_tx_clear_slot(&_tx, RS_SLOT_FAULT); }
    if (_cfg.persist_faults) {
        rs_ee_record_t ee = { 0, 0, { 0 } };
        df_write(ee);
    }
}

uint32_t BlinkoClass::bootCount() const { return blinko_fault_record.boot_count; }

bool BlinkoClass::flashSelfTest()
{
    rs_ee_record_t ee;
    bool r0 = df_read(ee);
    Serial.print("  read: "); Serial.print(r0 ? "ok" : "fail"); Serial.print(" magic=0x"); Serial.print(ee.magic, HEX);
    Serial.print(" text="); ee.text[RS_MSG_MAX_LEN] = 0; Serial.println(ee.text);
    rs_ee_record_t w = { 0x54455354u, 7, "selftest" };
    bool r1 = df_write(w);
    rs_ee_record_t back; bool r2 = df_read(back);
    Serial.print("  write: "); Serial.print(r1 ? "ok" : "fail"); Serial.print(" readback: "); Serial.print(r2 ? "ok" : "fail");
    Serial.print(" magic=0x"); Serial.print(back.magic, HEX); Serial.print(" text="); back.text[RS_MSG_MAX_LEN] = 0; Serial.println(back.text);
    df_write(ee);                                       /* what was there before the test, fault record included */
    return r1 && r2 && back.magic == 0x54455354u;
}

void BlinkoClass::checkpoint(const char *name)
{
    strncpy(blinko_fault_record.checkpoint, name, BLINKO_CHECKPOINT_LEN - 1);
    blinko_fault_record.checkpoint[BLINKO_CHECKPOINT_LEN - 1] = 0;
}

void BlinkoClass::_readResetCause()
{
    uint8_t r0 = R_SYSTEM->RSTSR0;
    uint16_t r1 = R_SYSTEM->RSTSR1;
    uint8_t r2 = R_SYSTEM->RSTSR2;
    const char *c = "PIN";
    if (r0 & (1u << R_SYSTEM_RSTSR0_PORF_Pos)) c = "POR";
    else if (r0 & 0x0E) c = "LVD";
    else if (r1 & (1u << R_SYSTEM_RSTSR1_IWDTRF_Pos)) c = "IWDT";
    else if (r1 & (1u << R_SYSTEM_RSTSR1_WDTRF_Pos)) c = "WDT";
    else if (r1 & (1u << R_SYSTEM_RSTSR1_SWRF_Pos)) c = "SW";
    else if (r1 & 0xFF00) c = "BUSERR";
    else if (r2 & (1u << R_SYSTEM_RSTSR2_CWSF_Pos)) c = "WARM";
    strncpy(_reset_cause, c, sizeof(_reset_cause) - 1);
    /* clear flags for next time (write 0 to flags) */
    R_SYSTEM->RSTSR0 = 0;
    R_SYSTEM->RSTSR1 = 0;
}

/* ------------------------------------------------------------- logging */

/* A message is packed with the chip interrupt running (rs_tx_slot_prepare) and only copied into
 * the transmitter with it masked: the packing takes several chip periods. */
void BlinkoClass::_setSlot(uint8_t id, uint8_t level, const char *text, size_t len)
{
    rs_slot_t slot;
    rs_tx_slot_prepare(&slot, level, text, len);
    /* The text that is on air already is left alone: a status() called on every loop() would
     * otherwise restart the slot from its first packet each time and never get through. */
    const rs_slot_t &cur = _tx.slots[id];
    if (slot.valid && cur.valid && cur.len == slot.len && cur.level == slot.level && cur.packed == slot.packed && memcmp(cur.data, slot.data, slot.len) == 0) return;
    IrqLock lock;
    rs_tx_put_slot(&_tx, id, &slot);
}

void BlinkoClass::_vlog(uint8_t level, const char *fmt, va_list ap)
{
    char buf[128];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    /* a long text becomes several messages, oldest piece first; each takes as much text as its
     * 31 bytes hold (up to 41 characters when the 6-bit packing applies) */
    for (int off = 0; off < n; ) {
        rs_slot_t slot;
        size_t took = rs_tx_slot_prepare(&slot, level, buf + off, (size_t)(n - off));
        if (took == 0) break;
        { IrqLock lock; rs_tx_log_slot(&_tx, &slot); }
        off += (int)took;
    }
}

void BlinkoClass::log(uint8_t level, const char *fmt, ...) { va_list ap; va_start(ap, fmt); _vlog(level, fmt, ap); va_end(ap); }
void BlinkoClass::debug(const char *fmt, ...) { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_DEBUG, fmt, ap); va_end(ap); }
void BlinkoClass::info(const char *fmt, ...)  { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_INFO, fmt, ap); va_end(ap); }
void BlinkoClass::warn(const char *fmt, ...)  { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_WARN, fmt, ap); va_end(ap); }
void BlinkoClass::error(const char *fmt, ...) { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_ERROR, fmt, ap); va_end(ap); }

size_t BlinkoClass::write(uint8_t c)
{
    if (c == '\n' || c == '\r') {
        if (_print_len) { _print_buf[_print_len] = 0; log(_print_level, "%s", _print_buf); _print_len = 0; }
        return 1;
    }
    if (_print_len < sizeof(_print_buf) - 1) _print_buf[_print_len++] = (char)c;
    return 1;
}
size_t BlinkoClass::write(const uint8_t *buf, size_t n) { for (size_t i = 0; i < n; i++) write(buf[i]); return n; }
void BlinkoClass::printf(const char *fmt, ...)
{
    char buf[128]; va_list ap; va_start(ap, fmt); int n = vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    if (n > 0) write((const uint8_t *)buf, (size_t)n);
}

void BlinkoClass::status(const char *fmt, ...)
{
    char buf[RS_TEXT_CHARS_MAX + 1];                    /* one message: what does not fit is dropped */
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    _setSlot(RS_SLOT_STATUS, RS_LVL_STATUS, buf, (size_t)n);
}

void BlinkoClass::fatal(uint8_t code, const char *fmt, ...)
{
    char buf[RS_MSG_MAX_LEN + 1];
    int p = snprintf(buf, sizeof(buf), "F%u:", (unsigned)code);
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf + p, sizeof(buf) - p, fmt, ap);
    va_end(ap);
    _persistAndLoop(buf);
}

/* ------------------------------------------------- fault transmission */

/* Record the fault text (RAM + data flash) and transmit forever
 * without relying on interrupts. Used by fatal() and the hard fault handler. */
void BlinkoClass::_persistAndLoop(const char *text)
{
    rs_fault_record_t &fr = blinko_fault_record;
    strncpy(fr.text, text, RS_MSG_MAX_LEN); fr.text[RS_MSG_MAX_LEN] = 0;
    fr.magic = BLINKO_FAULT_MAGIC;

    __disable_irq();
    if (Blinko._running) { s_timer.stop(); }

    /* Persist right now: the data-flash driver is blocking (no BGO, no IRQ),
     * so this works even from the hard fault handler. */
    if (Blinko._cfg.persist_faults) persist_fault(fr.text, fr.boot_count);

    /* Fresh transmitter: FAULT + STATUS + a copy of the recent log slots. */
    static rs_tx_t ftx;
    rs_tx_init(&ftx);
    for (uint8_t i = 0; i < RS_NUM_LOG_SLOTS; i++) ftx.slots[i] = Blinko._tx.slots[i];
    ftx.seq_counter = Blinko._tx.seq_counter;
    ftx.slots[RS_SLOT_STATUS] = Blinko._tx.slots[RS_SLOT_STATUS];
    rs_tx_set_slot(&ftx, RS_SLOT_FAULT, RS_LVL_FAULT, fr.text, strlen(fr.text));

    /* Red LED of death: single stream on the fault pin only, always pulsed (150/50 ms) so
     * a human sees a blinking red LED and a phone reads the reason inside the blink. Its
     * timing is the conservative fault_chip_us / fault_repeat (defaults T = 120 us, 3 copies),
     * not the running configuration: whoever picks the phone up must be able to read it. */
    uint32_t fault_t_us = Blinko._cfg.fault_chip_us >= BLINKO_MIN_CHIP_US ? Blinko._cfg.fault_chip_us : 120;   /* begin() may not have run */
    uint32_t cell_us = fault_t_us / RS_CELLS_PER_T;
    rs_tx_set_channels(&ftx, 1, 0);
    rs_tx_set_fault_weight(&ftx, Blinko._cfg.fault_weight);
    rs_tx_set_repeat(&ftx, Blinko._cfg.fault_repeat ? Blinko._cfg.fault_repeat : 3);
    rs_tx_set_burst(&ftx, 150000u / cell_us, 50000u / cell_us);
    /* Plain GPIO from here: no timers, no interrupts. The PWM objects are left as they are (the
     * pinMode below takes the pins away from their timers): closing them frees memory, and this
     * may be running in a hard fault raised by a corrupted heap. */
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) s_pfs_reg[c][k] = nullptr;
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++)
        if (Blinko._cfg.ch_pins[c][k] != BLINKO_NO_PIN) pinMode(Blinko._cfg.ch_pins[c][k], OUTPUT);
    Blinko._writeAll(0);
    pinMode(Blinko._cfg.fault_pin, OUTPUT);

    /* Exact chip timing from the DWT cycle counter. The work per chip (encoder +
     * GPIO write) must not add to the period: a chip that grows to more than twice
     * its length makes a packet taller than the LED blob in the camera frame and
     * nothing decodes. Falls back to the calibrated delay if the counter does not run. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t cyc = (SystemCoreClock / 1000000u) * cell_us;
    uint32_t probe = DWT->CYCCNT;
    R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
    bool have_dwt = (DWT->CYCCNT != probe) && cyc > 0;
    uint32_t next = DWT->CYCCNT + cyc;
    uint8_t chip = rs_tx_next_chip(&ftx);
    for (;;) {
        if (have_dwt) {
            while ((int32_t)(DWT->CYCCNT - next) < 0) { }
            next += cyc;
        } else {
            R_BSP_SoftwareDelay(cell_us, BSP_DELAY_UNITS_MICROSECONDS);
        }
        pin_write(Blinko._cfg.fault_pin, chip ^ Blinko._cfg.fault_pin_active_low);
        /* the packet after this one is encoded right after a packet starts, during the three
         * dark chips of its gap (the chip it makes late is a dark one after a dark one), then
         * the next chip while this one is being shown */
        if (rs_tx_wants_prepare(&ftx)) rs_tx_prepare(&ftx);
        chip = rs_tx_next_chip(&ftx);
    }
}

uint16_t BlinkoClass::boardId() const
{
    const bsp_unique_id_t *u = R_BSP_UniqueIdGet();      /* 128-bit factory id */
    uint32_t h = 2166136261u;
    for (int i = 0; i < 16; i++) { h ^= u->unique_id_bytes[i]; h *= 16777619u; }   /* FNV-1a */
    return (uint16_t)(h ^ (h >> 16));
}

/* ---------------------------------------------------- tuning helpers */

void BlinkoClass::setChipMicros(uint32_t us)
{
    if (us < BLINKO_MIN_CHIP_US) us = BLINKO_MIN_CHIP_US;
    _cfg.chip_us = us;
    if (_running) {
        _applyTiming();
        if (!_strobe) s_timer.set_frequency(1.0e6f / (float)cellMicros());
    }
}

/* Bursts and pilot interval are kept in milliseconds and handed to the transmitter in chips of
 * T / 3 (cellMicros): every change of T, of the bursts or of the channels goes through here.
 * (setBurst and setChannels once divided by T instead: bursts and pilot interval a third of
 * what was asked until the next setChipMicros.) */
void BlinkoClass::_applyTiming()
{
    uint32_t cell_us = cellMicros();
    IrqLock lock;
    rs_tx_set_burst(&_tx, (uint32_t)_cfg.burst_on_ms * 1000u / cell_us, (uint32_t)_cfg.burst_off_ms * 1000u / cell_us);
    rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / cell_us);
}

void BlinkoClass::setBurst(uint16_t on_ms, uint16_t off_ms)
{
    _cfg.burst_on_ms = on_ms; _cfg.burst_off_ms = off_ms;
    _applyTiming();
}

void BlinkoClass::setEnabled(bool on) { _enabled = on; if (!on) _writeAll(0); }

void BlinkoClass::strobe(float hz)
{
    if (hz <= 0) { _strobe = false; if (_running) s_timer.set_frequency(1.0e6f / (float)cellMicros()); return; }
    _strobe = true;
    if (_running) s_timer.set_frequency(2.0f * hz);   /* toggle twice per period */
}

void BlinkoClass::ledTest(bool on)
{
    _enabled = false;
    _writeAll(on ? 1 : 0);
}

void BlinkoClass::setChannels(uint8_t n)
{
    _cfg.channels = (n == 3) ? 3 : 1;
    _applyTiming();
}

void BlinkoClass::setRepeat(uint8_t n)
{
    _cfg.repeat = n < 1 ? 1 : (n > RS_TX_MAX_REPEAT ? RS_TX_MAX_REPEAT : n);
    IrqLock lock;
    rs_tx_set_repeat(&_tx, _cfg.repeat);
}
