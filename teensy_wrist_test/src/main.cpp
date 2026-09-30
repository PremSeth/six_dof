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
volatile bool paired = false;
volatile bool velocityMode = false;
volatile int rates[2] = {0, 0};
volatile uint32_t phases[2] = {0, 0};
volatile bool directionPause[2] = {false, false};
volatile uint32_t signedSteps[2] = {0, 0}; // Wrapped signed displacement.
volatile uint32_t totalSteps[2] = {0, 0};
volatile uint32_t lastEncoderMs[2] = {0, 0}, lastEncoderSteps[2] = {0, 0};
volatile bool haveEncoder[2] = {false, false};
volatile bool predictionEnabled = false;
volatile uint8_t outcome = 0; // 5 = prediction grace expired
constexpr uint32_t ENCODER_GRACE_MS = 250;
constexpr uint32_t BLIND_STEP_LIMIT = 88; // 0.99 degrees at3200ppr,10:1.
bool predictionRead = false;
const char* readFault = "none";

bool predictionExpired() {
  if (!predictionEnabled) return false;
  for (unsigned i = 0; i < 2; ++i) {
    if (!haveEncoder[i] || (uint32_t)(millis() - lastEncoderMs[i]) >= ENCODER_GRACE_MS ||
        (uint32_t)(totalSteps[i] - lastEncoderSteps[i]) >= BLIND_STEP_LIMIT) return true;
  }
  return false;
}

// Independent pulse rates on one 20 kHz tick; sign sets encoder-positive DIR1.
void velocityTick() {
  if (!moving || !velocityMode) return;
  // Enforce freshness even if the foreground is stuck in an I2C transaction.
  if (predictionExpired()) {
    pulseTimer.end();
    digitalWriteFast(0, LOW); digitalWriteFast(2, LOW);
    moving = false; velocityMode = false;
    rates[0] = rates[1] = 0;
    outcome = 5;
    return;
  }
  bool fire[2] = {false, false};
  for (unsigned i = 0; i < 2; ++i) {
    if (directionPause[i]) { directionPause[i] = false; continue; }
    phases[i] += abs(rates[i]);
    if (phases[i] >= 20000) { phases[i] -= 20000; fire[i] = true; }
  }
  if (fire[0]) digitalWriteFast(0, HIGH);
  if (fire[1]) digitalWriteFast(2, HIGH);
  if (fire[0] || fire[1]) delayMicroseconds(5);
  digitalWriteFast(0, LOW);
  digitalWriteFast(2, LOW);
  for (unsigned i = 0; i < 2; ++i) if (fire[i]) {
    signedSteps[i] += rates[i] < 0 ? UINT32_MAX : 1u;
    ++totalSteps[i];
  }
}
// outcome:0idle,1moving,2complete,3stop,4link loss,5encoder grace expired
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
  if (paired) digitalWriteFast(0, STEP_ACTIVE);
  digitalWriteFast(STEP_PIN, STEP_ACTIVE);
  delayMicroseconds(5);
  digitalWriteFast(STEP_PIN, STEP_IDLE);
  if (paired) digitalWriteFast(0, STEP_IDLE);
  ++generated;
  if (last) { moving = false; outcome = 2; }
}

void stop(uint8_t reason) {
  noInterrupts();
  moving = false;
  pulseTimer.end();
  digitalWriteFast(0, STEP_IDLE);
  velocityMode = false;
  rates[0] = rates[1] = 0;
  phases[0] = phases[1] = 0;
  paired = false;
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

bool wristReadFailure(uint8_t port, const char* stage, int code, int detail = -1) {
  readFault = stage;
  if (!predictionRead) stop(3); // Legacy clients retain immediate stop.
  Wire.beginTransmission(0x70);
  Wire.write((uint8_t)0);
  uint8_t cleanup = Wire.endTransmission();
  if (!predictionRead)
    Serial.printf("ERROR wrist_encoder_read port=%u stage=%s code=%d detail=%d cleanup=%u\n",
                  port, stage, code, detail, cleanup);
  return false;
}

bool readWristEncoder(uint8_t port, uint16_t& raw) {
  Wire.beginTransmission(0x70);
  Wire.write((uint8_t)(1u << port));
  uint8_t code = Wire.endTransmission();
  if (code) return wristReadFailure(port, "mux_select", code);
  delayMicroseconds(100); // Match the settling delay in I2C_SCAN.
  uint8_t received = Wire.requestFrom((uint8_t)0x70, (uint8_t)1);
  if (received != 1) return wristReadFailure(port, "mux_read_bytes", received, 1);
  int mask = Wire.read();
  if (mask != (1 << port)) return wristReadFailure(port, "mux_mask", mask, 1 << port);
  Wire.beginTransmission(0x36);
  Wire.write((uint8_t)0x0C);
  code = Wire.endTransmission(false);
  if (code) return wristReadFailure(port, "encoder_register", code);
  received = Wire.requestFrom((uint8_t)0x36, (uint8_t)2);
  if (received != 2) return wristReadFailure(port, "encoder_read_bytes", received, 2);
  raw = (Wire.read() & 15) << 8;
  raw |= Wire.read();
  Wire.beginTransmission(0x70);
  Wire.write((uint8_t)0);
  code = Wire.endTransmission();
  if (code) return wristReadFailure(port, "mux_deselect", code);
  return true;
}

bool signedRate(const char* s, int& result) {
  if (!s || !*s) return false;
  bool negative = *s == '-';
  if (negative) ++s;
  uint32_t n;
  if (!number(s, n) || n > 2000) return false;
  result = negative ? -(int)n : (int)n;
  return true;
}

void command() {
  if (!strcmp(line, "EST_CAPS")) {
    Serial.print("WRIST_EST_V1 GRACE_MS=250 BLIND_STEPS=");
    Serial.println(BLIND_STEP_LIMIT);
  } else if (!strcmp(line, "WRIST_EST")) {
    if (moving && !velocityMode) { Serial.println("ERROR busy"); return; }
    predictionEnabled = true;
    predictionRead = true;
    Wire.begin(); Wire.setClock(100000);
    uint16_t raw[2] = {0,0};
    bool valid[2];
    uint32_t counts[2];
    const char* faults[2];
    for (unsigned i = 0; i < 2; ++i) {
      readFault = "none";
      valid[i] = readWristEncoder(i+1, raw[i]);
      faults[i] = readFault;
      noInterrupts();
      counts[i] = signedSteps[i];
      if (valid[i]) {
        lastEncoderMs[i] = millis();
        lastEncoderSteps[i] = totalSteps[i];
        haveEncoder[i] = true;
      }
      interrupts();
    }
    predictionRead = false;
    lastContact = millis();
    Serial.printf("EST validA=%u rawA=%u stepsA=%lu validB=%u rawB=%u stepsB=%lu state=%u faultA=%s faultB=%s\n",
      valid[0],raw[0],(unsigned long)counts[0],valid[1],raw[1],(unsigned long)counts[1],outcome,faults[0],faults[1]);
  } else if (!strcmp(line, "WRIST_CAPS")) {
    Serial.println("WRIST_FB_V1 A=0,1,1 B=2,3,2 MAX_HZ=2000");
  } else if (!strcmp(line, "WRIST_READ")) {
    Wire.begin();
    Wire.setClock(100000);
    uint16_t a, b;
    if (!readWristEncoder(1, a) || !readWristEncoder(2, b)) return;
    lastContact = millis();
    Serial.printf("WRIST rawA=%u rawB=%u\n", a, b);
  } else if (!strncmp(line, "WRIST_VEL ", 10)) {
    if ((moving && !velocityMode) || testServos[0].attached() || testServos[1].attached()) {
      Serial.println("ERROR busy"); return;
    }
    char* save;
    char* a = strtok_r(line + 10, " ", &save);
    char* b = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    int newRates[2];
    if (!signedRate(a, newRates[0]) || !signedRate(b, newRates[1]) || extra) {
      Serial.println("ERROR expected_WRIST_VEL_signedHzA_signedHzB_max2000"); return;
    }
    lastContact = millis();
    if (!newRates[0] && !newRates[1]) { stop(3); Serial.println("OK WRIST_VEL"); return; }
    if (predictionExpired()) { stop(5); Serial.println("ERROR encoder_grace_expired"); return; }
    bool starting = !velocityMode;
    noInterrupts();
    for (unsigned i = 0; i < 2; ++i) {
      if (starting || (rates[i] >= 0) != (newRates[i] >= 0) || !newRates[i]) {
        phases[i] = 0;
        directionPause[i] = true; // At least one 50 us tick for DIR settling.
      }
      rates[i] = newRates[i];
    }
    digitalWriteFast(1, newRates[0] >= 0 ? HIGH : LOW);
    digitalWriteFast(3, newRates[1] >= 0 ? HIGH : LOW);
    velocityMode = true;
    paired = false;
    moving = true;
    outcome = 1;
    bool ok = !starting || pulseTimer.begin(velocityTick, 50);
    if (!ok) { moving = false; velocityMode = false; rates[0] = rates[1] = 0; outcome = 0; }
    interrupts();
    Serial.println(ok ? "OK WRIST_VEL" : "ERROR timer_unavailable");
  } else if (!strcmp(line, "PAIR_CAPS")) {
    Serial.println("PAIR A_STEP=0 A_DIR=1 B_STEP=2 B_DIR=3 MAX=800");
  } else if (!strncmp(line, "PAIR ", 5)) {
    if (moving || testServos[0].attached() || testServos[1].attached()) {
      Serial.println("ERROR busy"); return;
    }
    char* save;
    char* a = strtok_r(line + 5, " ", &save);
    char* b = strtok_r(nullptr, " ", &save);
    char* c = strtok_r(nullptr, " ", &save);
    char* d = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    uint32_t count, period, dirA, dirB;
    if (!number(a, count) || !number(b, period) || !number(c, dirA) ||
        !number(d, dirB) || extra || count < 1 || count > 800 ||
        period < 5000 || period > 100000 || dirA > 1 || dirB > 1) {
      Serial.println("ERROR expected_PAIR_count1to800_period5000to100000_dirA_dirB"); return;
    }
    digitalWriteFast(1, dirA ? HIGH : LOW);
    digitalWriteFast(DIR_PIN, dirB ? HIGH : LOW);
    delay(10);
    lastContact = millis();
    noInterrupts();
    generated = 0;
    requested = count;
    paired = true;
    moving = true;
    outcome = 1;
    bool ok = pulseTimer.begin(pulse, period);
    if (!ok) { moving = false; paired = false; outcome = 0; }
    interrupts();
    Serial.println(ok ? "OK PAIR" : "ERROR timer_unavailable");
  } else if (!strcmp(line, "SERVO_STATUS")) {
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
    Serial.println("READY SIX_DOF_WRIST_V1 STEP=2 DIR=3 INVERTED=0");
  } else if (!strcmp(line, "HOLD_STEP")) {
    if (testServos[0].attached() || testServos[1].attached()) { Serial.println("ERROR servos_active"); return; }
    if (moving) { Serial.println("ERROR busy"); return; }
    lastContact = millis();
    noInterrupts();
    requested = 1;
    paired = false;
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
    paired = false;
    moving = true;
    bool ok = pulseTimer.begin(pulse, period);
    if (!ok) { moving = false; outcome = 0; }
    interrupts();
    Serial.println(ok ? (continuous ? "OK RUN" : "OK MOVE") : "ERROR timer_unavailable");
  } else Serial.println("ERROR unknown_command");
}

void setup() {
  // Keep the other test motor's STEP line inactive while mapping this one.
  digitalWrite(0, LOW);
  pinMode(0, OUTPUT);
  digitalWrite(1, LOW);
  pinMode(1, OUTPUT);
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
