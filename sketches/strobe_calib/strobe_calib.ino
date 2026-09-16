/*
 * strobe_calib — bare strobe for measuring a camera's rolling shutter
 * (see docs/CALIBRATION.md). Blinks all LEDs with a square wave.
 * Serial: "f <hz>" sets the frequency (default 2000 Hz), "d <percent>" duty.
 */
#include <RSLog.h>

void setup()
{
    Serial.begin(115200);
    RSLogConfig cfg;
    cfg.announce_boot = false;
    cfg.persist_faults = false;
    RSLog.begin(cfg);
    RSLog.strobe(2000.0f);
}

void loop()
{
    static String line;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            line.trim();
            if (line.startsWith("f ")) { float hz = line.substring(2).toFloat(); RSLog.strobe(hz); Serial.print("strobe "); Serial.println(hz); }
            line = "";
        } else line += c;
    }
}
