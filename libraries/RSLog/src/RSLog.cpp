#include "RSLog.h"
#include <FspTimer.h>
#include <DataFlashBlockDevice.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RSLOG_FAULT_MAGIC 0x52534641u   /* "RSFA" */
#define RSLOG_BOOT_MAGIC  0x52534254u   /* "RSBT" */
#define RSLOG_EE_MAGIC    0x52534545u   /* "RSEE" */
#define RSLOG_DF_OFFSET   (8192 - 1024)   /* last 1 KB data-flash block (raw, no virtual-EEPROM layer) */
#define RSLOG_LOAD_MAGIC  0x4C4F4144u   /* "LOAD": set while reading flash at boot (boot-loop guard) */

RSLogClass RSLog;

/* Reset-surviving record. The .noinit section of the Arduino linker script sits
 * in low RAM, which the Nano R4 bootloader zeroes on every reset (verified on
 * hardware); high RAM survives. We use a fixed address just below the main
 * stack region (__StackLimit = 0x20007B00 on the R4 linker script). The heap
 * only reaches it after ~25 KB of allocations. */
#ifndef RSLOG_RECORD_ADDR
#define RSLOG_RECORD_ADDR 0x20007A00u
#endif
#define rslog_fault_record (*(rs_fault_record_t *)RSLOG_RECORD_ADDR)

static FspTimer s_timer;

struct rs_ee_record_t { uint32_t magic; uint32_t boot_count; char text[RS_MSG_MAX_LEN + 1]; };

/* Raw data-flash persistence. The FSP flash_lp driver is blocking (no BGO, no
 * IRQ), so these work from fatal() and from the hard fault handler. */
/* Note: the core's DataFlashBlockDevice::init() fails when already open and
 * read() reports an error code even on success, so we call erase/program/read
 * directly (they open the driver themselves) and validate by magic. */
static bool df_write(const rs_ee_record_t &r)
{
    DataFlashBlockDevice &bd = DataFlashBlockDevice::getInstance();
    if (bd.erase(RSLOG_DF_OFFSET, 1024) != 0) return false;
    return bd.program(&r, RSLOG_DF_OFFSET, sizeof(r)) == 0;
}
static bool df_read(rs_ee_record_t &r)
{
    DataFlashBlockDevice &bd = DataFlashBlockDevice::getInstance();
    (void)bd.read(&r, RSLOG_DF_OFFSET, sizeof(r));
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

static void timer_cb(timer_callback_args_t *) { RSLog._tick(); }

/* ---------------------------------------------------------------- pins */

void RSLogClass::_writeChips(const uint8_t chips[RS_MAX_CHANNELS])
{
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++) {
        for (uint8_t k = 0; k < RSLOG_PINS_PER_CH; k++) {
            uint8_t pin = _cfg.ch_pins[c][k];
            if (pin == RSLOG_NO_PIN) continue;
            digitalWrite(pin, (chips[c] ^ _cfg.ch_active_low[c][k]) ? HIGH : LOW);
        }
    }
}

void RSLogClass::_writeAll(uint8_t level)
{
    uint8_t v[RS_MAX_CHANNELS] = { level, level, level };
    _writeChips(v);
}

void RSLogClass::_tick()
{
    if (!_enabled) return;
    if (_strobe) { _strobe_level ^= 1; _writeAll(_strobe_level); return; }
    uint8_t chips[RS_MAX_CHANNELS];
    rs_tx_next_chips(&_tx, chips);
    _writeChips(chips);
}

/* --------------------------------------------------------------- timer */

bool RSLogClass::_startTimer(float hz)
{
    uint8_t type = GPT_TIMER;
    int8_t ch = FspTimer::get_available_timer(type);
    if (ch < 0) ch = FspTimer::get_available_timer(type, true);
    if (ch < 0) return false;
    if (!s_timer.begin(TIMER_MODE_PERIODIC, type, (uint8_t)ch, hz, 50.0f, timer_cb, nullptr)) return false;
    if (!s_timer.setup_overflow_irq()) return false;
    if (!s_timer.open()) return false;
    return s_timer.start();
}

/* --------------------------------------------------------------- begin */

bool RSLogClass::begin(const RSLogConfig &cfg)
{
    _cfg = cfg;
    rs_tx_init(&_tx);
    for (uint8_t c = 0; c < RS_MAX_CHANNELS; c++)
        for (uint8_t k = 0; k < RSLOG_PINS_PER_CH; k++)
            if (_cfg.ch_pins[c][k] != RSLOG_NO_PIN) pinMode(_cfg.ch_pins[c][k], OUTPUT);
    pinMode(_cfg.fault_pin, OUTPUT);
    _writeAll(0);

    _readResetCause();

    /* Warm reset bookkeeping */
    rs_fault_record_t &fr = rslog_fault_record;
    bool warm = (fr.boot_magic == RSLOG_BOOT_MAGIC);
    if (!warm) { fr.boot_magic = RSLOG_BOOT_MAGIC; fr.boot_count = 0; fr.checkpoint[0] = 0; fr.magic = 0; fr.loading = 0; }
    if (fr.fw_id != fw_build_id()) { fr.fw_id = fw_build_id(); fr.checkpoint[0] = 0; warm = false; /* fresh upload */ }
    fr.boot_count++;

    if (fr.magic == RSLOG_FAULT_MAGIC) {
        /* a fault was recorded before the reset: persist it and announce */
        fr.text[RS_MSG_MAX_LEN] = 0;
        strncpy(_fault_text, fr.text, RS_MSG_MAX_LEN); _fault_text[RS_MSG_MAX_LEN] = 0;
        fr.magic = 0;
        if (_cfg.persist_faults) {
            rs_ee_record_t ee = { RSLOG_EE_MAGIC, fr.boot_count, { 0 } };
            strncpy(ee.text, _fault_text, RS_MSG_MAX_LEN);
            df_write(ee);
        }
    } else if (warm && (strstr(_reset_cause, "WDT") || strstr(_reset_cause, "IWDT"))) {
        /* watchdog reset with no explicit record: report last checkpoint */
        fr.checkpoint[RSLOG_CHECKPOINT_LEN - 1] = 0;
        snprintf(_fault_text, sizeof(_fault_text), "WDT reset @%s", fr.checkpoint[0] ? fr.checkpoint : "?");
        if (_cfg.persist_faults) {
            rs_ee_record_t ee = { RSLOG_EE_MAGIC, fr.boot_count, { 0 } };
            strncpy(ee.text, _fault_text, RS_MSG_MAX_LEN);
            df_write(ee);
        }
    } else if (_cfg.persist_faults) {
        if (warm && fr.loading == RSLOG_LOAD_MAGIC) {
            /* the previous boot crashed while reading the record: don't retry, wipe it */
            rs_ee_record_t ee = { 0, 0, { 0 } };
            df_write(ee);
            strncpy(_fault_text, "boot-loop guard: record wiped", RS_MSG_MAX_LEN);
        } else {
            fr.loading = RSLOG_LOAD_MAGIC;
            _loadPersistedFault();
        }
    }
    fr.loading = 0;
    fr.checkpoint[0] = 0;

    if (_fault_text[0]) _setSlot(RS_SLOT_FAULT, RS_LVL_FAULT, _fault_text, strlen(_fault_text));

    if (_cfg.announce_boot) status("boot#%lu rst=%s", (unsigned long)fr.boot_count, _reset_cause);

    rs_tx_set_burst(&_tx, (uint32_t)_cfg.burst_on_ms * 1000u / _cfg.chip_us, (uint32_t)_cfg.burst_off_ms * 1000u / _cfg.chip_us);
    rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / _cfg.chip_us);
    _running = _startTimer(1.0e6f / (float)_cfg.chip_us);
    return _running;
}

void RSLogClass::end()
{
    if (_running) { s_timer.stop(); s_timer.end(); _running = false; }
    _writeAll(0);
}

void RSLogClass::_loadPersistedFault()
{
    rs_ee_record_t ee;
    if (!df_read(ee)) return;
    if (ee.magic == RSLOG_EE_MAGIC) {
        ee.text[RS_MSG_MAX_LEN] = 0;
        strncpy(_fault_text, ee.text, RS_MSG_MAX_LEN); _fault_text[RS_MSG_MAX_LEN] = 0;
    }
}

void RSLogClass::clearFault()
{
    _fault_text[0] = 0;
    noInterrupts(); rs_tx_clear_slot(&_tx, RS_SLOT_FAULT); interrupts();
    if (_cfg.persist_faults) {
        rs_ee_record_t ee = { 0, 0, { 0 } };
        df_write(ee);
    }
}

uint32_t RSLogClass::bootCount() const { return rslog_fault_record.boot_count; }

bool RSLogClass::flashSelfTest()
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

void RSLogClass::checkpoint(const char *name)
{
    strncpy(rslog_fault_record.checkpoint, name, RSLOG_CHECKPOINT_LEN - 1);
    rslog_fault_record.checkpoint[RSLOG_CHECKPOINT_LEN - 1] = 0;
}

void RSLogClass::_readResetCause()
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

void RSLogClass::_setSlot(uint8_t id, uint8_t level, const char *text, size_t len)
{
    noInterrupts();
    rs_tx_set_slot(&_tx, id, level, text, len);
    interrupts();
}

void RSLogClass::_vlog(uint8_t level, const char *fmt, va_list ap)
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

void RSLogClass::log(uint8_t level, const char *fmt, ...) { va_list ap; va_start(ap, fmt); _vlog(level, fmt, ap); va_end(ap); }
void RSLogClass::debug(const char *fmt, ...) { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_DEBUG, fmt, ap); va_end(ap); }
void RSLogClass::info(const char *fmt, ...)  { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_INFO, fmt, ap); va_end(ap); }
void RSLogClass::warn(const char *fmt, ...)  { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_WARN, fmt, ap); va_end(ap); }
void RSLogClass::error(const char *fmt, ...) { va_list ap; va_start(ap, fmt); _vlog(RS_LVL_ERROR, fmt, ap); va_end(ap); }

void RSLogClass::status(const char *fmt, ...)
{
    char buf[RS_MSG_MAX_LEN + 1];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > RS_MSG_MAX_LEN) n = RS_MSG_MAX_LEN;
    _setSlot(RS_SLOT_STATUS, RS_LVL_STATUS, buf, (size_t)n);
}

void RSLogClass::fatal(uint8_t code, const char *fmt, ...)
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
void RSLogClass::_persistAndLoop(const char *text)
{
    rs_fault_record_t &fr = rslog_fault_record;
    strncpy(fr.text, text, RS_MSG_MAX_LEN); fr.text[RS_MSG_MAX_LEN] = 0;
    fr.magic = RSLOG_FAULT_MAGIC;

    __disable_irq();
    if (RSLog._running) { s_timer.stop(); }

    /* Persist right now: the data-flash driver is blocking (no BGO, no IRQ),
     * so this works even from the hard fault handler. */
    if (RSLog._cfg.persist_faults) {
        rs_ee_record_t ee = { RSLOG_EE_MAGIC, fr.boot_count, { 0 } };
        strncpy(ee.text, fr.text, RS_MSG_MAX_LEN);
        df_write(ee);
    }

    /* Fresh transmitter: FAULT + STATUS + a copy of the recent log slots. */
    static rs_tx_t ftx;
    rs_tx_init(&ftx);
    for (uint8_t i = 0; i < RS_NUM_LOG_SLOTS; i++) ftx.slots[i] = RSLog._tx.slots[i];
    ftx.seq_counter = RSLog._tx.seq_counter;
    ftx.slots[RS_SLOT_STATUS] = RSLog._tx.slots[RS_SLOT_STATUS];
    rs_tx_set_slot(&ftx, RS_SLOT_FAULT, RS_LVL_FAULT, fr.text, strlen(fr.text));

    /* Red LED of death: single stream on the fault pin only, always pulsed (150/50 ms) so
     * a human sees a blinking red LED and a phone reads the reason inside the blink. */
    uint32_t chip_us = RSLog._cfg.chip_us ? RSLog._cfg.chip_us : 30;
    rs_tx_set_channels(&ftx, 1, 0);
    rs_tx_set_burst(&ftx, 150000u / chip_us, 50000u / chip_us);
    RSLog._writeAll(0);
    pinMode(RSLog._cfg.fault_pin, OUTPUT);
    for (;;) {
        uint8_t chip = rs_tx_next_chip(&ftx);
        digitalWrite(RSLog._cfg.fault_pin, (chip ^ RSLog._cfg.fault_pin_active_low) ? HIGH : LOW);
        R_BSP_SoftwareDelay(chip_us, BSP_DELAY_UNITS_MICROSECONDS);
    }
}

/* ---------------------------------------------------- tuning helpers */

void RSLogClass::setChipMicros(uint32_t us)
{
    if (us < 15) us = 15;
    _cfg.chip_us = us;
    noInterrupts();
    rs_tx_set_burst(&_tx, (uint32_t)_cfg.burst_on_ms * 1000u / us, (uint32_t)_cfg.burst_off_ms * 1000u / us);
    rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / us);
    interrupts();
    if (_running) s_timer.set_frequency(1.0e6f / (float)us);
}

void RSLogClass::setBurst(uint16_t on_ms, uint16_t off_ms)
{
    _cfg.burst_on_ms = on_ms; _cfg.burst_off_ms = off_ms;
    noInterrupts();
    rs_tx_set_burst(&_tx, (uint32_t)on_ms * 1000u / _cfg.chip_us, (uint32_t)off_ms * 1000u / _cfg.chip_us);
    interrupts();
}

void RSLogClass::setEnabled(bool on) { _enabled = on; if (!on) _writeAll(0); }

void RSLogClass::strobe(float hz)
{
    if (hz <= 0) { _strobe = false; if (_running) s_timer.set_frequency(1.0e6f / (float)_cfg.chip_us); return; }
    _strobe = true;
    if (_running) s_timer.set_frequency(2.0f * hz);   /* toggle twice per period */
}

void RSLogClass::ledTest(bool on)
{
    _enabled = false;
    _writeAll(on ? 1 : 0);
}

void RSLogClass::setChannels(uint8_t n)
{
    _cfg.channels = (n == 3) ? 3 : 1;
    noInterrupts();
    rs_tx_set_channels(&_tx, _cfg.channels, (uint32_t)_cfg.pilot_ms * 1000u / _cfg.chip_us);
    interrupts();
}
