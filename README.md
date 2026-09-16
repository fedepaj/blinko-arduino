# rslog-arduino

Arduino library `RSLog` (Renesas RA4M1: Nano R4 / UNO R4) plus demo sketches.
The shared core lives in the git submodule `core/`; `build.sh` copies the
encoder files into `libraries/RSLog/src/core/` so the library stays
self-contained for the Arduino IDE.

```sh
git submodule update --init
./build.sh rslog_demo upload      # PORT=/dev/cu.usbmodemXXXX to force the port
```
