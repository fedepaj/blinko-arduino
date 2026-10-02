# blinko-arduino

Arduino library `Blinko` for the **Arduino Nano R4** (Renesas RA4M1), with its
examples (`libraries/Blinko/examples/`, where the Arduino IDE also finds
them). The shared core lives in the git submodule `core/`; `build.sh` copies
the transmitter files into `libraries/Blinko/src/core/` so the library stays
self-contained for the Arduino IDE.

The wire format is described in `docs/PROTOCOL.md` of the
[blinko-core](https://github.com/fedepaj/blinko-core) repository (`core/docs/`
in this checkout). Choosing T for a camera is described in
`docs/CALIBRATION.md` of the umbrella repository,
[blinko](https://github.com/fedepaj/blinko).

## Requirements

An Arduino Nano R4: its on-board RGB LED and `LED_BUILTIN` are the
transmitter. The UNO R4 boards are not supported (their variants do not
define `LEDR`, `LEDG`, `LEDB`, which the default configuration uses).
`arduino-cli` with the Renesas core (`arduino-cli core install
arduino:renesas_uno`; default FQBN `arduino:renesas_uno:nanor4`, override with
`FQBN=...`), and a phone running the Blinko app.

## Build and run

```sh
git submodule update --init
./build.sh BlinkoDemo upload         # PORT=/dev/cu.usbmodemXXXX to force the port
./build.sh StrobeCalibration upload  # calibration strobe
```

`make build` / `make upload` / `make calib` are shortcuts. With several Nano
R4 connected, `PORT` is required and the upload goes through
`tools/flash_r4.py` of the umbrella repository. For the Arduino IDE, copy or
symlink `libraries/Blinko` into your sketchbook `libraries/` folder.

Open the serial monitor at 115200, type `info hello` and hold the phone 1–3 cm
from the LEDs.

## Minimal sketch

```cpp
#include <Blinko.h>
void setup() {
  Blinko.begin();                    // T = 60 µs, RGB LED + LED_BUILTIN
  Blinko.info("boot ok");
  Blinko.checkpoint("init-sensors"); // reported if a watchdog reset follows
  if (!sensor.begin()) Blinko.fatal(3, "sensor init");   // never returns
}
void loop() {
  static uint32_t last;
  if (millis() - last >= 5000) { last = millis(); Blinko.status("up=%lus", millis() / 1000); }
}
```

## API (`libraries/Blinko/src/Blinko.h`)

Units: **T** (`chip_us`, `setChipMicros`) is the shortest run of the line
code, 60 µs by default; the timer runs at T/3, one chip.

| Call | Effect |
|---|---|
| `begin(cfg = BlinkoConfig())` | claim the pins, restore any persisted fault, announce the boot, start the chip timer; `false` if the timer failed |
| `end()` | stop the timer and turn the LEDs off |
| `log(level, fmt, …)`, `debug/info/warn/error(fmt, …)` | queue a message in the log carousel (slots 0–5, the newest sent first); a seventh message replaces the oldest |
| `status(fmt, …)` | set the STATUS slot (6): uptime, mode, counters. One message; the same text again changes nothing |
| `fatal(code, fmt, …)` | record `F<code>:<text>` in RAM and flash and blink it on the fault LED forever; never returns |
| `checkpoint(name)` | name the current phase (up to 15 characters), reported as `WDT reset @name` if a watchdog reset follows |
| `print/println/printf(…)` | `Print` interface: every `\n`-terminated line becomes a message |
| `setPrintLevel(level)` | level used by the `Print` interface (default `RS_LVL_INFO`) |
| `hasFault()`, `faultText()` | whether the FAULT slot (7) holds a record, and its text |
| `clearFault()` | empty the FAULT slot and erase the persisted record |
| `boardId()` | 16-bit id from the MCU's unique id, announced as `id=xxxx` in the boot STATUS |
| `resetCause()` | cause of the last reset: `POR`, `PIN`, `LVD`, `WDT`, `IWDT`, `SW`, `BUSERR`, `WARM` |
| `bootCount()` | boots since power-on |
| `packetsSent()` | packets taken from the carousel so far |
| `setChipMicros(us)`, `chipMicros()` | set and read T; a value under 45 µs becomes 45 |
| `cellMicros()` | the timer period, T/3 |
| `setRepeat(n)`, `repeat()` | send every packet n times back to back (1–100): 2–3 for a phone whose blob is about one packet tall |
| `setBrightness(percent)`, `brightness()` | lit level 1–100 by a 240 kHz PWM on the LED pins (100 = plain on/off): lower it when the phone saturates |
| `setBurst(on_ms, off_ms)` | visible blink; `off_ms = 0` transmits continuously |
| `setChannels(n)`, `channels()` | 3 = independent RGB streams, 1 = one stream on every LED |
| `setEnabled(on)` | pause and resume the output |
| `ledTest(on)` | hold every LED on or off (polarity check); `setEnabled(true)` resumes |
| `strobe(hz)` | square wave for rolling-shutter calibration; `strobe(0)` returns to data |

Diagnostics, for bringing the library up on a board: `timerInfo()` (chip
timer type, channel and tick count), `pfsInfo()` (pin function register of the
red LED), `flashSelfTest()` (writes and reads back a scratch record in the
data-flash block, then restores what was there; prints to `Serial`), `tx()`
(the core transmitter).

Calls may come from `loop()` and from interrupt handlers; a message costs a
formatted print plus a short copy with interrupts masked.

## Messages

- A message holds 31 bytes: up to **41 characters** of ordinary log text (a–z,
  digits, space, common punctuation; an uppercase letter counts as two
  symbols), 31 of anything else. A longer `log()` text is split into several
  messages, 127 characters per call at most. `status()` is one message: what
  does not fit is dropped.
- `status()` with the text already on air is a no-op, so it can be called on
  every `loop()`. A new text restarts the slot: a status that changes faster
  than a phone reads it, about once a second, is not received.
- A text identical to the one a slot already delivered is not shown again by
  the receiver.
- **A fault persists.** The record of a hard fault, a `fatal()` or a watchdog
  reset is kept in RAM across the reset and in data flash across power cycles
  (`persist_faults`), and is sent again after every boot until `clearFault()`.

Fault texts:

| Text | Cause |
|---|---|
| `HF p=XXXXXXXX l=XXXXXXXX c=XXXX` | hard fault: program counter, link register, low 16 bits of the fault status register (CFSR), in hex. Up to three return addresses found on the stack follow as an ERROR message, `bt xxxxx xxxxx xxxxx` |
| `WDT reset @checkpoint` | watchdog reset, with the last `checkpoint()` name (`?` if none) |
| `F<code>:<text>` | `fatal(code, text)` |

While the board is dead only the fault LED transmits, pulsed 150/50 ms, at
`fault_chip_us` / `fault_repeat` (T = 120 µs, three copies), with the FAULT
slot taking about three quarters of the packets and the STATUS and the last
log messages the rest.

## What the library takes

- **One GPT timer** for the chips, and the **PendSV** exception, in which the
  next packets are encoded: a sketch cannot use PendSV for anything else.
- The **hard fault handler**.
- The **last 1 KB block of the data flash** (EEPROM addresses 7168..8191)
  while `persist_faults` is on: a sketch that uses `EEPROM` must stay below
  7168.
- **68 bytes of RAM at 0x20007A00**, just below the main stack, for the record
  that survives a reset. The heap reaches that address only after about 25 KB
  of allocations.
- The **GPT channels of the LED pins** (red 5A, green 6B, blue 6A, builtin 4B)
  while the brightness is under 100.

What it costs: the chip interrupt takes about 12 µs every T/3, so the library
uses roughly 60 % of the CPU at T = 60 µs and about 80 % at T = 45 µs, the
shortest T it runs (`BLINKO_MIN_CHIP_US`).

## `BlinkoConfig`

| Field | Default | Meaning |
|---|---|---|
| `chip_us` | `60` | T, the shortest run of the line code (µs): keep it above the phone's exposure; the timer runs at T/3. At least 45 |
| `repeat` | `1` | copies of every packet (1–100); 2–3 for a phone whose blob is about one packet tall (a 30 fps Android phone) |
| `brightness` | `100` | lit level in percent, by a 240 kHz PWM on the LED pins gated by the chips; 100 = plain on/off. Lower it when the phone's sensor saturates |
| `fault_chip_us` / `fault_repeat` | `120` / `3` | the death loop's own timing, a conservative setting independent of `chip_us` and `repeat` |
| `fault_weight` | `3` | FAULT visits per other visit in the death loop (1–4) |
| `fault_pin` / `fault_pin_active_low` | `LEDR` / `1` | the "red LED of death" |
| `ch_pins` / `ch_active_low` | `{LEDR, LED_BUILTIN}, {LEDG}, {LEDB}` | up to 2 pins per channel and their polarity |
| `channels` | `3` | 3 = RGB streams (3× throughput), 1 = same stream everywhere |
| `pilot_ms` | `30` | mean interval between RGB colour-calibration pilots (36 chips each: 2.4 % of the time at T = 60 µs) |
| `burst_on_ms` / `burst_off_ms` | `150` / `50` | visible blink pattern; `0` off-time = continuous |
| `persist_faults` | `true` | keep fault records in data flash across power cycles |
| `announce_boot` | `true` | fill STATUS with `boot#<n> rst=<cause> id=<board id>` at `begin()` |

## The demo sketch (`BlinkoDemo`)

It transmits continuously (no visible blink), logs `boot ok fw=0.1` at boot
and sets a STATUS `up=<s>s rst=<cause> n=<counter> id=<board id>` every 5 s.
Shorting **D2 to D3** causes a real hard fault: the red LED blinks the reason
until RESET. Serial commands, 115200 baud, one per line:

| Command | Effect |
|---|---|
| `info <text>`, `warn <text>`, `err <text>`, `debug <text>` | log a message at that level |
| `status <text>` | set the STATUS slot |
| `print <text>` | send `<text> n=<counter>` through the `Print` interface |
| `fatal <text>` | `Blinko.fatal(42, text)`: blinks `F42:<text>` forever (RESET to recover) |
| `hf` | provoke a real hard fault (a bus fault); a 4 s watchdog reboots the board, which then sends the persisted record |
| `hang` | stop serving a 2 s watchdog: the reset is reported at the next boot as `WDT reset @hang-test` |
| `clear` | clear the fault record |
| `chip <us>` | T in µs (default 60, at least 45) |
| `rep <n>` | copies of every packet, 1–100 |
| `bright <percent>` | LED brightness, 1–100 |
| `rgb 3` / `rgb 1` | three RGB streams, or one stream on every LED |
| `burst <on_ms> <off_ms>` | visible blink; off 0 = continuous |
| `strobe <hz>` | calibration square wave; `strobe 0` returns to data |
| `led on` / `led off` / `led data` | every LED steadily on or off (polarity check); `led data` resumes transmission |
| `stat` | packets sent, T, repeat, channels, brightness, reset cause, boot count, fault text, the slots in use |
| `reset` | software reset |
| `dftest` | data-flash self-test (`flashSelfTest()`) |
| `timer`, `pfs` | diagnostics (`timerInfo()`, `pfsInfo()`) |

`StrobeCalibration` is a bare strobe on every LED: 2000 Hz at start, `f <hz>`
on its serial port changes the frequency.
