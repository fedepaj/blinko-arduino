/*
 * strobe_calib — bare strobe for measuring a camera's rolling shutter (the procedure is in
 * docs/CALIBRATION.md of the blinko repository, the umbrella of this one). Blinks all LEDs
 * with a square wave.
 * Serial (115200): "f <hz>" sets the frequency (default 2000 Hz).
 */
#include <Blinko.h>

void setup()
{
    Serial.begin(115200);
    BlinkoConfig cfg;
    cfg.announce_boot = false;
    cfg.persist_faults = false;
    Blinko.begin(cfg);
    Blinko.strobe(2000.0f);
}

void loop()
{
    static String line;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            line.trim();
            if (line.startsWith("f ")) { float hz = line.substring(2).toFloat(); Blinko.strobe(hz); Serial.print("strobe "); Serial.println(hz); }
            line = "";
        } else line += c;
    }
}
