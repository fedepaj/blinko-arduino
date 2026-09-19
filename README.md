# blinko-arduino

Arduino library `Blinko` (Renesas RA4M1: Nano R4 / UNO R4) with its examples
(`libraries/Blinko/examples/`, where the Arduino IDE also finds them).
The shared core lives in the git submodule `core/`; `build.sh` copies the
encoder files into `libraries/Blinko/src/core/` so the library stays
self-contained for the Arduino IDE.

## Requirements

Arduino Nano R4 or UNO R4 (the on-board RGB LED and `LED_BUILTIN` are the
transmitter), `arduino-cli` with the Renesas core
(`arduino-cli core install arduino:renesas_uno`; default FQBN
`arduino:renesas_uno:nanor4`, override with `FQBN=...`), and a phone running
the Blinko viewer.

## Build and run

```sh
git submodule update --init
./build.sh BlinkoDemo upload         # PORT=/dev/cu.usbmodemXXXX to force the port
./build.sh StrobeCalibration upload  # calibration strobe, see docs/CALIBRATION.md
```

`make build` / `make upload` / `make calib` are shortcuts. Open the serial
monitor at 115200 and try `info hello`, `warn x`, `status up`, `fatal y`,
`hf`, `hang`, `clear`, `chip 30`, `rgb 3|1`, `burst 150 50`, `strobe 2000`,
`led on|off|data`, `print x`, `stat`, `reset`; hold the phone 1–3 cm from the
LEDs. Shorting D2 to D3 causes a real hard fault: the red LED blinks the
reason until RESET. For the Arduino IDE, copy or symlink `libraries/Blinko`
into your sketchbook `libraries/` folder.

## Minimal sketch

```cpp
#include <Blinko.h>
void setup() {
  Blinko.begin();                    // 30 µs/chip, RGB LED + LED_BUILTIN
  Blinko.info("boot ok");
  Blinko.checkpoint("init-sensors"); // reported if a watchdog reset follows
  if (!sensor.begin()) Blinko.fatal(3, "sensor init");   // never returns
}
void loop() { Blinko.status("up=%lus", millis() / 1000); }
```

## API (`libraries/Blinko/src/Blinko.h`)

| Call | Effect |
|---|---|
| `begin(cfg = BlinkoConfig())` | claim the pins, restore any persisted fault, start the chip timer; `false` if the timer failed |
| `end()` | stop the timer and turn the LEDs off |
| `log(level, fmt, …)`, `debug/info/warn/error(fmt, …)` | queue a message in the log carousel (slots 0–5); text longer than 31 bytes is split |
| `status(fmt, …)` | overwrite the STATUS slot (6): uptime, mode, counters |
| `fatal(code, fmt, …)` | record `F<code>:<text>` in RAM + flash and blink it forever; never returns |
| `checkpoint(name)` | name the current phase (≤ 15 chars), reported as `WDT reset @name` |
| `print/println/printf(…)` | `Print` interface: every `\n`-terminated line becomes a message |
| `setPrintLevel(level)` | level used by the `Print` interface (default `RS_LVL_INFO`) |
| `boardId()` | 16-bit id from the MCU unique id, announced as `id=xxxx` |
| `hasFault()`, `faultText()`, `clearFault()` | state of the FAULT slot (7) and of the persisted record |
| `setChipMicros(us)` / `chipMicros()` | chip duration, clamped to ≥ 15 µs (re-tunes burst and pilots) |
| `setBurst(on_ms, off_ms)` | visible blink; `off_ms = 0` transmits continuously |
| `setChannels(n)` | 3 = independent RGB streams, 1 = one stream on every LED |
| `setEnabled(on)`, `ledTest(on)` | pause/resume the output, or hold every LED on (polarity check) |
| `strobe(hz)` | square wave for rolling-shutter calibration; `strobe(0)` returns to data |
| `packetsSent()`, `resetCause()`, `bootCount()`, `flashSelfTest()` | diagnostics |

## `BlinkoConfig`

| Field | Default | Meaning |
|---|---|---|
| `chip_us` | `30` | half-bit duration; must be ≥ 4 camera row times (see `docs/CALIBRATION.md`) |
| `ch_pins` / `ch_active_low` | `{LEDR, LED_BUILTIN}, {LEDG}, {LEDB}` | up to 2 pins per channel and their polarity |
| `channels` | `3` | 3 = RGB streams (3× throughput), 1 = same stream everywhere |
| `pilot_ms` | `30` | interval between RGB colour-calibration pilots (36 chips each, 3.6 % overhead) |
| `fault_weight` | `3` | FAULT visits per other visit in the death loop (1–4) |
| `burst_on_ms` / `burst_off_ms` | `150` / `50` | visible blink pattern; `0` off-time = continuous |
| `fault_pin` / `fault_pin_active_low` | `LEDR` / `1` | the "red LED of death" |
| `persist_faults` | `true` | keep fault records in data flash across power cycles |
| `announce_boot` | `true` | fill STATUS with boot count, reset cause and board id at `begin()` |
