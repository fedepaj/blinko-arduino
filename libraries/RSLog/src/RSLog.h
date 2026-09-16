/*
 * RSLog — optical logger for rolling-shutter cameras (Arduino Nano R4 / UNO R4).
 *
 *   #include <RSLog.h>
 *   void setup() { RSLog.begin(); RSLog.info("boot ok"); }
 *
 * Messages are transmitted continuously by blinking the on-board LEDs from a
 * hardware timer interrupt. A phone camera pointed at the board decodes them.
 * See docs/PROTOCOL.md for the wire format.
 */
#ifndef RSLOG_H
#define RSLOG_H

#include <Arduino.h>
#include "core/rs_proto.h"
#include "core/rs_tx.h"

#define RSLOG_PINS_PER_CH 2
#define RSLOG_NO_PIN 0xFF
#define RSLOG_CHECKPOINT_LEN 16

struct RSLogConfig {
    uint32_t chip_us = 30;                       /* half-bit duration; iPhone 14: 5 us/row -> ~6 rows/chip */
    /* LEDs per channel (RGB streams). Channel 0 = red (+ LED_BUILTIN mirrors it), 1 = green, 2 = blue. */
    uint8_t  ch_pins[RS_MAX_CHANNELS][RSLOG_PINS_PER_CH] = { { LEDR, LED_BUILTIN }, { LEDG, RSLOG_NO_PIN }, { LEDB, RSLOG_NO_PIN } };
    uint8_t  ch_active_low[RS_MAX_CHANNELS][RSLOG_PINS_PER_CH] = { { 1, 0 }, { 1, 0 }, { 1, 0 } };
    uint8_t  channels = 3;                       /* 3 = independent RGB streams (3x throughput), 1 = all LEDs same stream */
    uint16_t pilot_ms = 100;                     /* RGB colour-calibration pilots interval */
    uint16_t burst_on_ms = 150;                  /* visible blink: transmit for burst_on_ms ... */
    uint16_t burst_off_ms = 50;                  /* ... then dark for burst_off_ms. 0 = continuous (user choice) */
    uint8_t  fault_pin = LEDR;                   /* the "red LED of death": fault records blink here, always pulsed */
    uint8_t  fault_pin_active_low = 1;
    bool     persist_faults = true;              /* keep fault records in data flash across power cycles */
    bool     announce_boot = true;               /* fill STATUS slot with reset cause at begin() */
};

/* Record that survives resets (placed in .noinit RAM). */
struct rs_fault_record_t {
    uint32_t magic;                              /* RSLOG_FAULT_MAGIC when a fault record is pending */
    uint32_t boot_magic;                         /* RSLOG_BOOT_MAGIC once begin() ran (detects warm resets) */
    uint32_t boot_count;
    uint32_t loading;                            /* boot-loop guard while reading flash */
    uint32_t fw_id;                              /* build id of the firmware that wrote the record */
    char     checkpoint[RSLOG_CHECKPOINT_LEN];
    char     text[RS_MSG_MAX_LEN + 1];
};

class RSLogClass {
public:
    bool begin(const RSLogConfig &cfg = RSLogConfig());
    void end();

    /* Logging (slots 0..5, newest first in the carousel). Text > 31 bytes is split. */
    void log(uint8_t level, const char *fmt, ...);
    void debug(const char *fmt, ...);
    void info(const char *fmt, ...);
    void warn(const char *fmt, ...);
    void error(const char *fmt, ...);

    /* STATUS slot (6): periodic state, e.g. uptime, mode, counters. */
    void status(const char *fmt, ...);

    /* FATAL: record the reason (RAM + EEPROM), then transmit forever. Never returns. */
    void fatal(uint8_t code, const char *fmt, ...) __attribute__((noreturn));

    /* Fault slot (7) management. */
    bool hasFault() const { return _tx.slots[RS_SLOT_FAULT].valid; }
    const char *faultText() const { return _fault_text; }
    void clearFault();

    /* Name the current phase; reported if a watchdog/hard fault reset follows. */
    void checkpoint(const char *name);

    /* Tuning / diagnostics */
    void setChipMicros(uint32_t us);
    void setBurst(uint16_t on_ms, uint16_t off_ms);   /* visible blink; off_ms = 0 -> continuous */
    void setChannels(uint8_t n);                       /* 3 = RGB streams, 1 = all LEDs same stream */
    uint32_t chipMicros() const { return _cfg.chip_us; }
    void setEnabled(bool on);
    void strobe(float hz);                       /* calibration square wave; strobe(0) resumes data */
    void ledTest(bool on);                       /* all LEDs steady on/off (polarity check) */
    uint32_t packetsSent() const { return _tx.packets_sent; }
    const char *resetCause() const { return _reset_cause; }
    uint32_t bootCount() const;
    bool flashSelfTest();                        /* write+read back a scratch record (diagnostics) */
    rs_tx_t &tx() { return _tx; }

    /* internal */
    void _tick();
    void _writeChips(const uint8_t chips[RS_MAX_CHANNELS]);
    void _writeAll(uint8_t level);
    static void _persistAndLoop(const char *text) __attribute__((noreturn));

private:
    void _vlog(uint8_t level, const char *fmt, va_list ap);
    void _setSlot(uint8_t id, uint8_t level, const char *text, size_t len);
    void _loadPersistedFault();
    void _readResetCause();
    bool _startTimer(float hz);

    RSLogConfig _cfg;
    rs_tx_t _tx;
    char _fault_text[RS_MSG_MAX_LEN + 1] = { 0 };
    char _reset_cause[12] = "?";
    bool _running = false;
    bool _enabled = true;
    bool _strobe = false;
    uint8_t _strobe_level = 0;
};

extern RSLogClass RSLog;

#endif
