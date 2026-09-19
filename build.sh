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
    PORT=${PORT:-$(arduino-cli board list 2>/dev/null | awk '/nanor4|Nano R4/ {print $1; exit}')}
    if [ -z "$PORT" ]; then
        PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    fi
    [ -n "$PORT" ] || { echo "no Nano R4 port found (set PORT=...)" >&2; exit 2; }
    arduino-cli upload --fqbn "$FQBN" -p "$PORT" --input-dir "$HERE/build/$SKETCH" "$HERE/libraries/Blinko/examples/$SKETCH"
fi
