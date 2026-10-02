/*
 * Blinko — optical logger for rolling-shutter cameras (Arduino Nano R4, Renesas RA4M1).
 *
 *   #include <Blinko.h>
 *   void setup() { Blinko.begin(); Blinko.info("boot ok"); }
 *
 * Messages are transmitted continuously by blinking the on-board LEDs from a
 * hardware timer interrupt. A phone camera pointed at the board decodes them.
 * The wire format is in docs/PROTOCOL.md of the blinko-core repository.
 *
 * What the library takes: one GPT timer for the chips (and the LED pins' GPT channels while the
 * brightness is under 100), the last 1 KB block of the data flash (EEPROM addresses 7168..8191)
 * when persist_faults is on, and 68 bytes of RAM at 0x20007A00 that survive a reset.
 * Calls from the sketch may come from loop() and from interrupt handlers; a message costs a
 * formatted print plus a short masked copy.
 */
#ifndef BLINKO_H
#define BLINKO_H

#include <Arduino.h>
#include "core/rs_proto.h"
#include "core/rs_tx.h"

#define BLINKO_PINS_PER_CH 2
#define BLINKO_NO_PIN 0xFF
#define BLINKO_CHECKPOINT_LEN 16
/* Shortest T. The chip interrupt (timer period T/3) takes about 12 us on the RA4M1: measured, the
 * board keeps its tick rate and answers on serial down to T = 39 us and stops answering at 36.
 * At T = 45 it leaves the sketch about a fifth of the CPU, at the default 60 about 40 %. */
#define BLINKO_MIN_CHIP_US 45
#define RS_TEXT_CHARS_MAX 41                        /* characters one message holds at most (31 bytes of 6-bit symbols) */

struct BlinkoConfig {
    uint32_t chip_us = 60;                       /* minimum run T of the line code (RLL(2,7)): keep >= the phone's exposure; the timer runs at T / RS_CELLS_PER_T */
    /* LEDs per channel (RGB streams). Channel 0 = red (+ LED_BUILTIN mirrors it), 1 = green, 2 = blue. */
    uint8_t  ch_pins[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH] = { { LEDR, LED_BUILTIN }, { LEDG, BLINKO_NO_PIN }, { LEDB, BLINKO_NO_PIN } };
    uint8_t  ch_active_low[RS_MAX_CHANNELS][BLINKO_PINS_PER_CH] = { { 1, 0 }, { 1, 0 }, { 1, 0 } };
    uint8_t  channels = 3;                       /* 3 = independent RGB streams (3x throughput), 1 = all LEDs same stream */
    uint16_t pilot_ms = 30;                      /* mean interval of the RGB colour-calibration pilot blocks (36 chips each: 2.4 % of the airtime at T = 60 us) */
    uint8_t  repeat = 1;                         /* send every packet n times back to back: phones whose window is shorter than a packet (30 fps Android) read it across two copies */
    uint8_t  brightness = 100;                   /* lit level in percent through a ~240 kHz PWM on the LED pins (100 = plain on/off): a phone a centimetre away saturates on a full-brightness LED and loses the short gaps; 30-50 % keeps the stripes in range */
    uint32_t fault_chip_us = 120;                /* death loop timing: the most conservative values that decoded on every phone tried (a 57 us-exposure Android needs T >= 90 us and 2-3 copies; the iPhone loses little): T = 120 us ... */
    uint8_t  fault_repeat = 3;                   /* ... and 3 copies of every packet, so a 30 fps phone reads a packet across two copies */
    uint8_t  fault_weight = 3;                   /* death loop: FAULT visits per other visit (1..4); 3 = fault reason in ~1-2 s */
    uint16_t burst_on_ms = 150;                  /* visible blink: transmit for burst_on_ms ... */
    uint16_t burst_off_ms = 50;                  /* ... then dark for burst_off_ms. 0 = continuous (user choice) */
    uint8_t  fault_pin = LEDR;                   /* the "red LED of death": fault records blink here, always pulsed */
    uint8_t  fault_pin_active_low = 1;
    bool     persist_faults = true;              /* keep fault records in data flash across power cycles */
    bool     announce_boot = true;               /* fill STATUS slot with reset cause at begin() */
};

/* Record that survives resets (at a fixed address in high RAM, see BLINKO_RECORD_ADDR in Blinko.cpp). */
struct rs_fault_record_t {
    uint32_t magic;                              /* BLINKO_FAULT_MAGIC when a fault record is pending */
    uint32_t boot_magic;                         /* BLINKO_BOOT_MAGIC once begin() ran (detects warm resets) */
    uint32_t boot_count;
    uint32_t loading;                            /* boot-loop guard while reading flash */
    uint32_t fw_id;                              /* build id of the firmware that wrote the record */
    char     checkpoint[BLINKO_CHECKPOINT_LEN];
    char     text[RS_MSG_MAX_LEN + 1];
};

class BlinkoClass : public Print {
public:
    /* Print interface: Blinko.print()/println()/printf() work like Serial; every line ('\n')
     * becomes a message at printLevel (INFO by default). Useful to redirect existing prints. */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buf, size_t n) override;
    void printf(const char *fmt, ...);
    void setPrintLevel(uint8_t level) { _print_level = level; }
    bool begin(const BlinkoConfig &cfg = BlinkoConfig());
    void end();

    /* Logging (slots 0..5, newest first in the carousel). A message holds 31 bytes: up to 41
     * characters of ordinary log text, 31 of anything else. A longer text is split into several
     * messages (127 characters per call at most); a seventh message replaces the oldest. */
    void log(uint8_t level, const char *fmt, ...);
    void debug(const char *fmt, ...);
    void info(const char *fmt, ...);
    void warn(const char *fmt, ...);
    void error(const char *fmt, ...);

    /* STATUS slot (6): periodic state, e.g. uptime, mode, counters. One message: what does not
     * fit is dropped. Calling it again with the same text changes nothing; a new text restarts
     * the slot, so a status that changes faster than a phone reads it (about a second) is
     * never seen. */
    void status(const char *fmt, ...);

    /* FATAL: record the reason (RAM + data flash), then blink it on the fault LED until reset.
     * Never returns. The reason is sent again after every boot until clearFault(). */
    void fatal(uint8_t code, const char *fmt, ...) __attribute__((noreturn));

    /* Fault slot (7) management. */
    bool hasFault() const { return _tx.slots[RS_SLOT_FAULT].valid; }
    const char *faultText() const { return _fault_text; }
    void clearFault();

    /* 16-bit id derived from the MCU's unique id (announced as "id=xxxx" in the boot STATUS,
     * shown by the app next to the light that sends it). */
    uint16_t boardId() const;

    /* Name the current phase; reported as "WDT reset @name" if a watchdog reset follows. */
    void checkpoint(const char *name);

    /* Tuning / diagnostics */
    void setChipMicros(uint32_t us);
    void setBurst(uint16_t on_ms, uint16_t off_ms);   /* visible blink; off_ms = 0 -> continuous */
    void setChannels(uint8_t n);                       /* 3 = RGB streams, 1 = all LEDs same stream */
    void setRepeat(uint8_t n);                         /* 1..100 copies of every packet: 2-3 for 30 fps phones, 20-60 for far lights (stitching) */
    void setBrightness(uint8_t percent);               /* 1..100: lit level by PWM (see BlinkoConfig::brightness); 100 = plain on/off */
    uint8_t brightness() const { return _cfg.brightness; }
    uint8_t repeat() const { return _cfg.repeat; }
    uint8_t channels() const { return _cfg.channels; }
    const char *timerInfo();                           /* diagnostics: chip timer type/channel, running, PWM active */
    const char *pfsInfo();                             /* diagnostics: red LED pin function register, lit / dark / current */
    uint32_t chipMicros() const { return _cfg.chip_us; }
    uint32_t cellMicros() const { return _cfg.chip_us / RS_CELLS_PER_T; }   /* timer period: one code cell */
    void setEnabled(bool on);
    void strobe(float hz);                       /* calibration square wave; strobe(0) resumes data */
    void ledTest(bool on);                       /* all LEDs steady on/off (polarity check) */
    uint32_t packetsSent() const { return _tx.packets_sent; }
    const char *resetCause() const { return _reset_cause; }
    uint32_t bootCount() const;
    bool flashSelfTest();                        /* write and read back a scratch record, then put back what was there (diagnostics; prints to Serial) */
    rs_tx_t &tx() { return _tx; }

    /* internal */
    void _tick();
    void _writeChips(const uint8_t chips[RS_MAX_CHANNELS]);
    void _writeAll(uint8_t level);
    void _applyBrightness();
    void _applyTiming();
    static void _persistAndLoop(const char *text) __attribute__((noreturn));

private:
    void _vlog(uint8_t level, const char *fmt, va_list ap);
    void _setSlot(uint8_t id, uint8_t level, const char *text, size_t len);
    void _loadPersistedFault();
    void _readResetCause();
    bool _startTimer(float hz);

    BlinkoConfig _cfg;
    rs_tx_t _tx;
    char _fault_text[RS_MSG_MAX_LEN + 1] = { 0 };
    char _print_buf[RS_MSG_MAX_LEN * 2 + 1];
    uint8_t _print_len = 0, _print_level = RS_LVL_INFO;
    char _reset_cause[12] = "?";
    bool _running = false;
    volatile bool _enabled = true;
    volatile bool _strobe = false;
    uint8_t _strobe_level = 0;
    uint8_t _next_chips[RS_MAX_CHANNELS] = { 0, 0, 0 };   /* what the next tick writes to the pins */
};

extern BlinkoClass Blinko;

#endif
