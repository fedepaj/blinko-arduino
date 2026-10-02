/*
 * blinko_demo — interactive demo of the Blinko optical logger.
 *
 * Point the phone app at the board. The sketch transmits continuously, logs "boot ok fw=0.1"
 * at boot and sets a STATUS "up=..s rst=.. n=.. id=...." every 5 s.
 * Serial (115200) commands, one per line:
 *   info <text> | warn <text> | err <text> | debug <text>   log a message
 *   status <text>        set the STATUS slot
 *   print <text>         Print interface: the line "<text> n=<counter>" becomes a message
 *   fatal <text>         Blinko.fatal(42, text): blink "F42:<text>" forever (RESET to recover)
 *   hf                   provoke a real hard fault (bus fault): the handler transmits it, a 4 s
 *                        watchdog reboots the board and the persisted record is sent again
 *   hang                 stop serving a 2 s watchdog: WDT reset, reported at the next boot
 *   clear                clear the fault record
 *   chip <us>            T, the shortest run of the line code (default 60 us, at least 45; the timer runs at T/3)
 *   rep <n>              copies of every packet, 1..100 (default 1)
 *   bright <percent>     lit level by PWM, 1..100 (default 100): lower it when the phone saturates
 *   rgb 3|1              RGB streams (default) or single stream on all LEDs
 *   burst <on> <off>     visible blink in ms (off 0 = continuous)
 *   strobe <hz>          calibration square wave; "strobe 0" resumes
 *   led on|off|data      steady LEDs (polarity check); "led data" resumes transmission
 *   stat                 print the transmitter state
 *   reset                software reset
 *   dftest               data-flash self-test
 *   timer | pfs          diagnostics: chip timer, pin function register of the red LED
 *
 * Settings for the two phones the project is measured on, board a few cm from the camera:
 *   iPhone 14, 120 fps, 15 us exposure:        chip 60, rep 1    (about 470 distinct packets/s)
 *   Samsung S21 FE, 30 fps RAW, 57 us exposure: chip 105, rep 3   (about 23 distinct packets/s)
 *   a phone that saturates on the LED:          bright 30..50, or move back
 * Choosing T for another camera: docs/CALIBRATION.md in the blinko repository (the umbrella of
 * this one). The red LED of death does not use these settings: it always sends at T = 120 us
 * with 3 copies (BlinkoConfig fault_chip_us / fault_repeat).
 *
 * Hardware demo: short D2 to D3 -> real hard fault -> red LED of death
 * (recover with RESET; the record is persisted and sent again after every boot until "clear").
 */
#include <Blinko.h>
#include <WDT.h>

static bool     g_wdt_on = false;
static bool     g_hang = false;
static uint32_t g_last_status = 0;
static uint32_t g_counter = 0;

static void handle(String line)
{
    line.trim();
    int sp = line.indexOf(' ');
    String cmd = sp < 0 ? line : line.substring(0, sp);
    String arg = sp < 0 ? "" : line.substring(sp + 1);
    cmd.toLowerCase();

    if (cmd == "info")        Blinko.info("%s", arg.c_str());
    else if (cmd == "warn")   Blinko.warn("%s", arg.c_str());
    else if (cmd == "err")    Blinko.error("%s", arg.c_str());
    else if (cmd == "debug")  Blinko.debug("%s", arg.c_str());
    else if (cmd == "status") Blinko.status("%s", arg.c_str());
    else if (cmd == "fatal")  { Serial.println("fatal: blinking forever"); Blinko.fatal(42, "%s", arg.c_str()); }
    else if (cmd == "hf") {
        /* watchdog so the board reboots by itself after ~4 s of fault blinking,
         * exercising the persisted-record path too (without a WDT it blinks forever) */
        if (!g_wdt_on) { WDT.begin(4000); g_wdt_on = true; }
        Serial.println("provoking hard fault (WDT reboot in ~4 s)"); Serial.flush();
        Blinko.checkpoint("hf-test");
        volatile uint32_t *bad = (volatile uint32_t *)0xCFFFFFF0u;  /* unmapped -> bus fault -> hard fault */
        (void)*bad;
    }
    else if (cmd == "hang") {
        if (!g_wdt_on) { WDT.begin(2000); g_wdt_on = true; }
        Blinko.checkpoint("hang-test");
        Serial.println("hanging: WDT will reset in ~2 s");
        g_hang = true;
    }
    else if (cmd == "clear")  { Blinko.clearFault(); Serial.println("fault cleared"); }
    else if (cmd == "dftest") { Serial.println(Blinko.flashSelfTest() ? "data flash ok" : "data flash FAILED"); }
    else if (cmd == "reset")  { Serial.println("software reset"); Serial.flush(); delay(50); NVIC_SystemReset(); }
    else if (cmd == "burst")  { int sp2 = arg.indexOf(' '); Blinko.setBurst(arg.toInt(), sp2 > 0 ? arg.substring(sp2 + 1).toInt() : 0); Serial.println("burst set"); }
    else if (cmd == "rgb")    { Blinko.setChannels(arg.toInt() == 1 ? 1 : 3); Serial.println(arg.toInt() == 1 ? "1 channel" : "3 channels (RGB)"); }
    else if (cmd == "rep")    { Blinko.setRepeat(arg.toInt()); Serial.print("repeat="); Serial.println(Blinko.repeat()); }
    else if (cmd == "bright") { Blinko.setBrightness(arg.toInt()); Serial.print("brightness="); Serial.println(Blinko.brightness()); }
    else if (cmd == "chip")   { Blinko.setChipMicros(arg.toInt()); Serial.print("chip_us="); Serial.println(Blinko.chipMicros()); }
    else if (cmd == "strobe") { Blinko.strobe(arg.toFloat()); Serial.println(arg.toFloat() > 0 ? "strobe on" : "data mode"); }
    else if (cmd == "print")  { Blinko.printf("%s n=%lu\n", arg.c_str(), (unsigned long)g_counter); }   /* Print interface: a line -> a message */
    else if (cmd == "led")    { arg.toLowerCase(); if (arg == "on") Blinko.ledTest(true); else if (arg == "off") Blinko.ledTest(false); else Blinko.setEnabled(true); }
    else if (cmd == "timer")  { Serial.println(Blinko.timerInfo()); }
    else if (cmd == "pfs")    { Serial.println(Blinko.pfsInfo()); }
    else if (cmd == "stat") {
        Serial.print("packets_sent="); Serial.println(Blinko.packetsSent());
        Serial.print("chip_us="); Serial.println(Blinko.chipMicros());
        Serial.print("repeat="); Serial.print(Blinko.repeat()); Serial.print(" channels="); Serial.print(Blinko.channels()); Serial.print(" brightness="); Serial.println(Blinko.brightness());
        Serial.print("reset="); Serial.println(Blinko.resetCause());
        Serial.print("boot#"); Serial.println(Blinko.bootCount());
        Serial.print("fault="); Serial.println(Blinko.hasFault() ? Blinko.faultText() : "(none)");
        rs_tx_t &tx = Blinko.tx();
        for (int i = 0; i < RS_NUM_SLOTS; i++) {
            if (!tx.slots[i].valid) continue;
            Serial.print("  slot "); Serial.print(i); Serial.print(" lvl "); Serial.print(tx.slots[i].level);
            Serial.print(tx.slots[i].packed ? " packed " : " raw "); Serial.print(tx.slots[i].len); Serial.println(" bytes");
        }
    }
    else if (cmd.length()) Serial.println("? commands: info warn err debug print status fatal hf hang clear chip rep rgb bright burst strobe led stat timer pfs dftest reset");
}

void setup()
{
    Serial.begin(115200);
    BlinkoConfig cfg;               /* defaults: T = 60 us, RGB streams + LED_BUILTIN */
    cfg.burst_off_ms = 0;          /* demo: continuous transmission (the fault loop always pulses the red LED) */
    Blinko.begin(cfg);
    pinMode(2, INPUT_PULLUP);      /* D2 pulled up ... */
    pinMode(3, OUTPUT); digitalWrite(3, LOW);   /* ... D3 low: shorting D2-D3 pulls D2 low */
    Blinko.info("boot ok fw=0.1");
    Blinko.checkpoint("setup");
    delay(1500);
    Serial.println("Blinko demo ready. reset=" + String(Blinko.resetCause()) + " boot#" + String(Blinko.bootCount()));
    if (Blinko.hasFault()) { Serial.print("persisted fault: "); Serial.println(Blinko.faultText()); }
}

void loop()
{
    static String line;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (line.length()) handle(line); line = ""; }
        else line += c;
    }
    if (g_hang) { for (;;) { } }             /* watchdog test */
    if (digitalRead(2) == LOW) {             /* D2 shorted to D3: die for real */
        Blinko.checkpoint("d2-d3 short");
        volatile uint32_t *bad = (volatile uint32_t *)0xCFFFFFF0u;
        (void)*bad;
    }
    if (g_wdt_on) WDT.refresh();
    if (millis() - g_last_status > 5000) {
        g_last_status = millis();
        Blinko.status("up=%lus rst=%s n=%lu id=%04x", (unsigned long)(millis() / 1000), Blinko.resetCause(), (unsigned long)g_counter++, Blinko.boardId());
        Blinko.checkpoint("loop");
    }
}
