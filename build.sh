#!/bin/sh
# Build (and optionally upload) a sketch for the Arduino Nano R4.
#   ./build.sh [example=BlinkoDemo] [upload]      PORT=/dev/cu.usbmodemXXX to force the port
# Examples live in libraries/Blinko/examples/ so the Arduino IDE lists them too.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SKETCH=${1:-BlinkoDemo}
FQBN=${FQBN:-arduino:renesas_uno:nanor4}
# keep the library's copy of the shared core (git submodule ./core) in sync
if [ -f "$HERE/core/rs_proto.h" ]; then
    cp "$HERE"/core/rs_proto.h "$HERE"/core/rs_tx.h "$HERE"/core/rs_tx.c "$HERE"/core/rs_pack.h "$HERE"/core/rs_pack.c "$HERE"/libraries/Blinko/src/core/
fi
mkdir -p "$HERE/build"
arduino-cli compile --fqbn "$FQBN" --libraries "$HERE/libraries" --output-dir "$HERE/build/$SKETCH" \
    --warnings default "$HERE/libraries/Blinko/examples/$SKETCH"
if [ "$2" = "upload" ]; then
    # several boards connected: flash by USB serial (arduino-cli's dfu-util refuses two DFU devices)
    N=$(arduino-cli board list 2>/dev/null | grep -c 'Nano R4')
    if [ "$N" -gt 1 ] && [ -f "$HERE/../tools/flash_r4.py" ]; then
        PY=${PY:-$HERE/../.venv/bin/python}
        [ -n "$PORT" ] || { echo "several Nano R4 connected: set PORT=/dev/cu.usbmodemXXX" >&2; exit 2; }
        exec "$PY" "$HERE/../tools/flash_r4.py" --port "$PORT" --bin "$HERE/build/$SKETCH/$SKETCH.ino.bin"
    fi
    PORT=${PORT:-$(arduino-cli board list 2>/dev/null | awk '/nanor4|Nano R4/ {print $1; exit}')}
    if [ -z "$PORT" ]; then
        PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    fi
    [ -n "$PORT" ] || { echo "no Nano R4 port found (set PORT=...)" >&2; exit 2; }
    arduino-cli upload --fqbn "$FQBN" -p "$PORT" --input-dir "$HERE/build/$SKETCH" "$HERE/libraries/Blinko/examples/$SKETCH"
fi
