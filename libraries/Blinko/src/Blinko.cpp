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

/* Diagnostics: the pin function register of the red LED as configured for lit / dark and now. */
const char *BlinkoClass::pfsInfo()
{
    static char buf[80];
    volatile uint32_t *reg = s_pfs_reg[0][0];
    snprintf(buf, sizeof(buf), "pfs on %08lx off %08lx now %08lx", (unsigned long)s_pfs_on[0][0], (unsigned long)s_pfs_off[0][0], reg ? (unsigned long)*reg : 0ul);
    return buf;
}

void BlinkoClass::_writeChips(const uint8_t chips[RS_MAX_CHANNELS])
{
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) {
        for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++) {
            uint8_t pin = _cfg.ch_pins[c][k];
            if (pin == BLINKO_NO_PIN) continue;
            if (s_pfs_reg[c][k]) pfs_write(s_pfs_reg[c][k], chips[c] ? s_pfs_on[c][k] : s_pfs_off[c][k]);
            else digitalWrite(pin, (chips[c] ^ _cfg.ch_active_low[c][k]) ? HIGH : LOW);
        }
    }
}

/* Brightness below 100 % puts every LED pin on a hardware PWM (the Nano R4's LED pins sit on
 * GPT channels: red 5A, green 6B, blue 6A, builtin 4B) at a fixed duty; the chips then switch
 * the pin between that output and a GPIO at the off level (see _writeChips). */
void BlinkoClass::_applyBrightness()
{
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
            if (!s_pwm[c][k]) {
                s_pwm[c][k] = new PwmOut(pin);
                if (!s_pwm[c][k]->begin(BLINKO_PWM_HZ, duty)) { delete s_pwm[c][k]; s_pwm[c][k] = nullptr; s_pfs_reg[c][k] = nullptr; continue; }
            } else {
                s_pwm[c][k]->pulse_perc(duty);
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
    if (was_running) _running = _startTimer(1.0e6f / (float)cellMicros());   /* (a strobe in progress resumes as data) */
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
    uint8_t chips[RS_MAX_CHANNELS];
    rs_tx_next_chips(&_tx, chips);
    _writeChips(chips);
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
    _cfg = cfg;
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
        if (_cfg.persist_faults) {
            rs_ee_record_t ee = { BLINKO_EE_MAGIC, fr.boot_count, { 0 } };
            strncpy(ee.text, _fault_text, RS_MSG_MAX_LEN);
            df_write(ee);
        }
    } else if (warm && (strstr(_reset_cause, "WDT") || strstr(_reset_cause, "IWDT"))) {
        /* watchdog reset with no explicit record: report last checkpoint */
        fr.checkpoint[BLINKO_CHECKPOINT_LEN - 1] = 0;
        snprintf(_fault_text, sizeof(_fault_text), "WDT reset @%s", fr.checkpoint[0] ? fr.checkpoint : "?");
        if (_cfg.persist_faults) {
            rs_ee_record_t ee = { BLINKO_EE_MAGIC, fr.boot_count, { 0 } };
            strncpy(ee.text, _fault_text, RS_MSG_MAX_LEN);
            df_write(ee);
        }
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

    uint32_t cell_us = cellMicros();
    rs_tx_set_burst(&_tx, (uint32_t)_cfg.burst_on_ms * 1000u / cell_us, (uint32_t)_cfg.burst_off_ms * 1000u / cell_us);
    rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / cell_us);
    rs_tx_set_repeat(&_tx, _cfg.repeat);
    _running = _startTimer(1.0e6f / (float)cell_us);
    return _running;
}

void BlinkoClass::end()
{
    if (_running) { s_timer.stop(); s_timer.end(); _running = false; }
    _writeAll(0);
}

void BlinkoClass::_loadPersistedFault()
{
    rs_ee_record_t ee;
    if (!df_read(ee)) return;
    if (ee.magic == BLINKO_EE_MAGIC) {
        ee.text[RS_MSG_MAX_LEN] = 0;
        strncpy(_fault_text, ee.text, RS_MSG_MAX_LEN); _fault_text[RS_MSG_MAX_LEN] = 0;
    }
}

void BlinkoClass::clearFault()
{
    _fault_text[0] = 0;
    noInterrupts(); rs_tx_clear_slot(&_tx, RS_SLOT_FAULT); interrupts();
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
    rs_ee_record_t z = { 0, 0, { 0 } }; df_write(z);
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

void BlinkoClass::_setSlot(uint8_t id, uint8_t level, const char *text, size_t len)
{
    noInterrupts();
    rs_tx_set_slot(&_tx, id, level, text, len);
    interrupts();
}

void BlinkoClass::_vlog(uint8_t level, const char *fmt, va_list ap)
{
    char buf[128];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    /* split long text into 31-byte messages, oldest chunk first */
    for (int off = 0; off < n || (off == 0 && n == 0); off += RS_MSG_MAX_LEN) {
        int len = n - off; if (len > RS_MSG_MAX_LEN) len = RS_MSG_MAX_LEN;
        if (len <= 0) break;
        noInterrupts();
        rs_tx_log(&_tx, level, buf + off, (size_t)len);
        interrupts();
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
    char buf[RS_MSG_MAX_LEN + 1];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > RS_MSG_MAX_LEN) n = RS_MSG_MAX_LEN;
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
    if (Blinko._cfg.persist_faults) {
        rs_ee_record_t ee = { BLINKO_EE_MAGIC, fr.boot_count, { 0 } };
        strncpy(ee.text, fr.text, RS_MSG_MAX_LEN);
        df_write(ee);
    }

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
    uint32_t chip_us = (Blinko._cfg.fault_chip_us ? Blinko._cfg.fault_chip_us : 120) / RS_CELLS_PER_T;   /* cell period */
    rs_tx_set_channels(&ftx, 1, 0);
    rs_tx_set_fault_weight(&ftx, Blinko._cfg.fault_weight);
    rs_tx_set_repeat(&ftx, Blinko._cfg.fault_repeat ? Blinko._cfg.fault_repeat : 3);
    rs_tx_set_burst(&ftx, 150000u / chip_us, 50000u / chip_us);
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++)
        { s_pfs_reg[c][k] = nullptr; if (s_pwm[c][k]) { s_pwm[c][k]->end(); s_pwm[c][k] = nullptr; } }   /* plain GPIO from here: no timers, no interrupts */
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) for (uint8_t k = 0; k < BLINKO_PINS_PER_CH; k++)
        if (Blinko._cfg.ch_pins[c][k] != BLINKO_NO_PIN) pinMode(Blinko._cfg.ch_pins[c][k], OUTPUT);
    Blinko._writeAll(0);
    pinMode(Blinko._cfg.fault_pin, OUTPUT);

    /* Exact chip timing from the DWT cycle counter. The work per chip (encoder +
     * GPIO write) must not add to the period: a chip that grows from 30 to 80 us
     * makes a packet taller than the LED blob in the camera frame and nothing
     * decodes. Falls back to the calibrated delay if the counter does not run. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t cyc = (SystemCoreClock / 1000000u) * chip_us;
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
            R_BSP_SoftwareDelay(chip_us, BSP_DELAY_UNITS_MICROSECONDS);
        }
        digitalWrite(Blinko._cfg.fault_pin, (chip ^ Blinko._cfg.fault_pin_active_low) ? HIGH : LOW);
        chip = rs_tx_next_chip(&ftx);           /* prepared while the chip is being shown */
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
    if (us < 24) us = 24;                          /* T >= 24 us: the timer cell (T/3) stays >= 8 us */
    _cfg.chip_us = us;
    if (_running) {
        uint32_t cell_us = cellMicros();
        rs_tx_set_burst(&_tx, (uint32_t)_cfg.burst_on_ms * 1000u / cell_us, (uint32_t)_cfg.burst_off_ms * 1000u / cell_us);
        rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / cell_us);
        s_timer.set_frequency(1.0e6f / (float)cell_us);
    }
}

void BlinkoClass::setBurst(uint16_t on_ms, uint16_t off_ms)
{
    _cfg.burst_on_ms = on_ms; _cfg.burst_off_ms = off_ms;
    noInterrupts();
    rs_tx_set_burst(&_tx, (uint32_t)on_ms * 1000u / _cfg.chip_us, (uint32_t)off_ms * 1000u / _cfg.chip_us);
    interrupts();
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
    noInterrupts();
    rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / _cfg.chip_us);
    interrupts();
}

void BlinkoClass::setRepeat(uint8_t n)
{
    _cfg.repeat = n < 1 ? 1 : (n > 100 ? 100 : n);
    noInterrupts(); rs_tx_set_repeat(&_tx, _cfg.repeat); interrupts();
}
