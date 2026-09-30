#include <Arduino.h>
#include <IntervalTimer.h>

// USB diagnostics only. No motor, encoder, servo or other external pins driven.
IntervalTimer timer;
volatile uint32_t ticks = 0;
volatile uint32_t scratch[16384]; // Test a dedicated 64 KiB region, not all RAM.
bool timerOK = false;
char line[256];
size_t used = 0;
bool overflow = false;

void onTick() { ++ticks; }

void handleLine() {
  if (!strcmp(line, "PING")) {
    Serial.println("READY SIX_DOF_TEENSY40_DIAG_V1");
  } else if (!strcmp(line, "STATUS")) {
    Serial.printf("STATUS uptime_ms=%lu ticks=%lu timer_ok=%u cpu_hz=%lu\n",
                  (unsigned long)millis(), (unsigned long)ticks,
                  timerOK, (unsigned long)F_CPU_ACTUAL);
  } else if (!strcmp(line, "RAMTEST")) {
    uint32_t errors = 0;
    for (unsigned pass = 0; pass < 4; ++pass) {
      uint32_t mask = 0x55555555u * pass;
      for (uint32_t i = 0; i < 16384; ++i) scratch[i] = (i * 2654435761u) ^ mask;
      for (uint32_t i = 0; i < 16384; ++i)
        if (scratch[i] != ((i * 2654435761u) ^ mask)) ++errors;
    }
    Serial.printf("RAMTEST bytes=65536 passes=4 errors=%lu\n", (unsigned long)errors);
  } else if (!strncmp(line, "ECHO ", 5)) {
    Serial.println(line);
  } else {
    Serial.println("ERROR unknown_command");
  }
}

void setup() {
  Serial.begin(115200);
  timerOK = timer.begin(onTick, 1000);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (overflow) Serial.println("ERROR line_too_long");
      else { line[used] = 0; handleLine(); }
      used = 0;
      overflow = false;
    } else if (c != '\r' && !overflow) {
      if (used < sizeof(line) - 1) line[used++] = c;
      else overflow = true;
    }
  }
}
