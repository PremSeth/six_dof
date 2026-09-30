#include <Arduino.h>
#include <IntervalTimer.h>
#include <Wire.h>
#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>

constexpr uint8_t STEP_PIN = 2, DIR_PIN = 3;
// Base TB6600 direct positive inputs: GPIO HIGH activates STEP.
constexpr uint8_t STEP_IDLE = LOW, STEP_ACTIVE = HIGH;
constexpr uint32_t LINK_TIMEOUT_MS = 1500;
IntervalTimer pulseTimer;
volatile uint32_t generated = 0, requested = 0;
volatile bool moving = false;
volatile uint8_t outcome = 0; // 0 idle, 1 moving, 2 complete, 3 stop, 4 link loss
uint32_t lastContact = 0;
char line[96];
size_t used = 0;
bool overflow = false;
Servo testServos[2];
constexpr uint8_t SERVO_PINS[2] = {14, 15};
uint32_t servoContact = 0;

void servosOff() {
  noInterrupts();
  for (unsigned i = 0; i < 2; ++i) {
    if (testServos[i].attached()) testServos[i].detach();
    digitalWrite(SERVO_PINS[i], LOW);
  }
  interrupts();
}

void finishHold() {
  pulseTimer.end();
  digitalWriteFast(STEP_PIN, STEP_IDLE);
  moving = false;
  outcome = 2;
}

void pulse() {
  if (!moving) return;
  const bool last = requested != 0 && generated + 1 >= requested;
  if (last) pulseTimer.end();
  digitalWriteFast(STEP_PIN, STEP_ACTIVE);
  delayMicroseconds(5);
  digitalWriteFast(STEP_PIN, STEP_IDLE);
  ++generated;
  if (last) { moving = false; outcome = 2; }
}

void stop(uint8_t reason) {
  noInterrupts();
  moving = false;
  pulseTimer.end();
  digitalWriteFast(STEP_PIN, STEP_IDLE);
  outcome = reason;
  interrupts();
}

bool number(const char* s, uint32_t& result) {
  if (!s || !*s) return false;
  uint64_t n = 0;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return false;
    n = n * 10 + (*s - '0');
    if (n > 10000000) return false;
  }
  result = (uint32_t)n;
  return true;
}

void command() {
  if (!strcmp(line, "SERVO_STATUS")) {
    servoContact = millis();
    Serial.printf("SERVOS pin14=%d pin15=%d\n",
      testServos[0].attached() ? testServos[0].readMicroseconds() : 0,
      testServos[1].attached() ? testServos[1].readMicroseconds() : 0);
  } else if (!strcmp(line, "SERVOS_OFF")) {
    servosOff();
    Serial.println("OK SERVOS_OFF");
  } else if (!strncmp(line, "SERVO ", 6)) {
    if (moving) { Serial.println("ERROR stepper_busy"); return; }
    char* save;
    char* a = strtok_r(line + 6, " ", &save);
    char* b = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    uint32_t pin, width;
    if (!number(a, pin) || !number(b, width) || extra ||
        (pin != 14 && pin != 15) || (width != 0 && (width < 500 || width > 2500))) {
      Serial.println("ERROR expected_SERVO_14_or_15_pulse_500_to_2500_or_0"); return;
    }
    unsigned i = pin - 14;
    servoContact = millis();
    noInterrupts();
    if (width == 0) {
      if (testServos[i].attached()) testServos[i].detach();
      digitalWrite(pin, LOW);
    } else {
      if (!testServos[i].attached()) testServos[i].attach(pin, 500, 2500);
      testServos[i].writeMicroseconds(width);
    }
    interrupts();
    Serial.println("OK SERVO");
  } else if (!strcmp(line, "ANGLE_BASE")) {
    lastContact = millis();
    Wire.begin();
    Wire.setClock(100000);
    Wire.beginTransmission(0x70);
    Wire.write((uint8_t)1); // BASE is now mux port 0.
    bool ok = Wire.endTransmission() == 0;
    if (ok) {
      ok = Wire.requestFrom((uint8_t)0x70, (uint8_t)1) == 1;
      if (ok) ok = Wire.read() == 1;
    }
    if (ok) {
      Wire.beginTransmission(0x36);
      Wire.write((uint8_t)0x0C); // RAW_ANGLE; no magnet-status gating.
      ok = Wire.endTransmission(false) == 0;
      if (ok) ok = Wire.requestFrom((uint8_t)0x36, (uint8_t)2) == 2;
    }
    uint16_t raw = 0;
    if (ok) { raw = (Wire.read() & 15) << 8; raw |= Wire.read(); }
    Wire.beginTransmission(0x70);
    Wire.write((uint8_t)0);
    ok = (Wire.endTransmission() == 0) && ok;
    if (!ok) { stop(3); Serial.println("ERROR encoder_read_failed"); }
    else Serial.printf("ANGLE_BASE port=0 raw=%u\n", raw);
  } else if (!strcmp(line, "I2C_SCAN") || !strcmp(line, "I2C_SCAN0")) {
    const uint8_t channelCount = !strcmp(line, "I2C_SCAN0") ? 1 : 8;
    if (moving) { Serial.println("ERROR busy"); return; }
    Wire.begin();
    Wire.setClock(100000);
    Serial.println("I2C SDA=18 SCL=19 speed=100000");
    Wire.beginTransmission(0x70);
    Wire.write((uint8_t)0);
    uint8_t muxResult = Wire.endTransmission();
    Serial.printf("MUX 0x70 result=%u (0=ACK)\n", muxResult);
    if (muxResult == 0) {
      for (uint8_t channel = 0; channel < channelCount; ++channel) {
        Wire.beginTransmission(0x70);
        Wire.write((uint8_t)(1u << channel));
        uint8_t selected = Wire.endTransmission();
        if (selected != 0) {
          Serial.printf("PORT %u select_error=%u\n", channel, selected);
          break;
        }
        delayMicroseconds(100);
        uint8_t maskBytes = Wire.requestFrom((uint8_t)0x70, (uint8_t)1);
        int mask = maskBytes == 1 ? Wire.read() : -1;
        if (mask != (1 << channel)) {
          Serial.printf("PORT %u mux_readback_error mask=%d\n", channel, mask);
          break;
        }
        Wire.beginTransmission(0x36);
        uint8_t ack = Wire.endTransmission();
        if (ack != 0) {
          Serial.printf("PORT %u AS5600 result=%u\n", channel, ack);
          continue;
        }
        Wire.beginTransmission(0x36);
        Wire.write((uint8_t)0x0B); // STATUS followed by RAW_ANGLE high/low.
        uint8_t regResult = Wire.endTransmission(false);
        uint8_t received = regResult == 0 ? Wire.requestFrom((uint8_t)0x36, (uint8_t)3) : 0;
        if (received == 3) {
          uint8_t status = Wire.read();
          uint16_t raw = (Wire.read() & 0x0F) << 8;
          raw |= Wire.read();
          Serial.printf("PORT %u AS5600 ACK raw=%u angle=%.2f MD=%u ML=%u MH=%u\n",
                        channel, raw, raw * (360.0 / 4096), !!(status & 0x20),
                        !!(status & 0x10), !!(status & 0x08));
        } else Serial.printf("PORT %u AS5600 ACK read_error=%u bytes=%u\n", channel, regResult, received);
      }
      Wire.beginTransmission(0x70);
      Wire.write((uint8_t)0);
      Serial.printf("MUX deselect_result=%u\n", Wire.endTransmission());
    }
    Serial.println("I2C_DONE");
  } else if (!strcmp(line, "PING")) {
    lastContact = millis();
    Serial.println("READY SIX_DOF_BASE_V1 STEP=2 DIR=3 INVERTED=0");
  } else if (!strcmp(line, "HOLD_STEP")) {
    if (testServos[0].attached() || testServos[1].attached()) { Serial.println("ERROR servos_active"); return; }
    if (moving) { Serial.println("ERROR busy"); return; }
    lastContact = millis();
    noInterrupts();
    requested = 1;
    generated = 0;
    moving = true;
    outcome = 1;
    bool ok = pulseTimer.begin(finishHold, 10000000u);
    if (ok) {
      digitalWriteFast(STEP_PIN, STEP_ACTIVE);
      generated = 1;
    } else { moving = false; outcome = 0; }
    interrupts();
    Serial.println(ok ? "OK HOLD_STEP 10s" : "ERROR timer_unavailable");
  } else if (!strcmp(line, "STOP")) {
    stop(3);
    Serial.println("OK STOP");
  } else if (!strcmp(line, "STATUS")) {
    lastContact = millis();
    noInterrupts();
    uint32_t count = generated, goal = requested;
    uint8_t state = outcome;
    interrupts();
    Serial.printf("STATUS state=%u pulses=%lu target=%lu\n", state,
                  (unsigned long)count, (unsigned long)goal);
  } else if (!strcmp(line, "CAPS")) {
    Serial.println("CAPS RUN ANGLE_BASE_PORT0");
  } else if (!strncmp(line, "MOVE ", 5) || !strncmp(line, "RUN ", 4)) {
    if (testServos[0].attached() || testServos[1].attached()) { Serial.println("ERROR servos_active"); return; }
    if (moving) { Serial.println("ERROR busy"); return; }
    const bool continuous = !strncmp(line, "RUN ", 4);
    char* save;
    char* a = strtok_r(line + (continuous ? 4 : 5), " ", &save);
    char* b = strtok_r(nullptr, " ", &save);
    char* c = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    uint32_t count = 0, period = 0, dir = 0;
    bool valid = continuous
      ? number(a, period) && number(b, dir) && !c && !extra
      : number(a, count) && count > 0 && number(b, period) && number(c, dir) && !extra;
    if (!valid || period < 50 || period > 1000000 || dir > 1) {
      Serial.println("ERROR expected_MOVE_count_period_dir_or_RUN_period_dir"); return;
    }
    digitalWriteFast(DIR_PIN, dir ? HIGH : LOW);
    delay(10); // DIR settles before the first pulse.
    lastContact = millis();
    noInterrupts();
    generated = 0;
    requested = count;
    outcome = 1;
    moving = true;
    bool ok = pulseTimer.begin(pulse, period);
    if (!ok) { moving = false; outcome = 0; }
    interrupts();
    Serial.println(ok ? (continuous ? "OK RUN" : "OK MOVE") : "ERROR timer_unavailable");
  } else Serial.println("ERROR unknown_command");
}

void setup() {
  digitalWriteFast(STEP_PIN, STEP_IDLE);
  digitalWriteFast(DIR_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  for (uint8_t pin : SERVO_PINS) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
}

void loop() {
  if ((testServos[0].attached() || testServos[1].attached()) &&
      (uint32_t)(millis() - servoContact) > LINK_TIMEOUT_MS) servosOff();
  // Serial bool stays false for 15 ms after opening even when commands arrive.
  // Use received heartbeats to detect loss, including unplugged USB.
  if (moving && (uint32_t)(millis() - lastContact) > LINK_TIMEOUT_MS)
    stop(4);
  // Bound parsing work so continuous input cannot starve the link watchdog.
  for (unsigned budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\n') {
      if (overflow) Serial.println("ERROR line_too_long");
      else { line[used] = 0; command(); }
      used = 0; overflow = false;
    } else if (c != '\r' && !overflow) {
      if (used < sizeof(line) - 1) line[used++] = c;
      else overflow = true;
    }
  }
}
