// Nontok SSF: Mega firmware. Wheel speed control, sensors and safety; the Jetson decides
// where to go. Protocol: PLAN.md section 6.3.
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "drivers.h"
#include "nontok_core.h"

using namespace nontok;

enum Fault : uint8_t { FAULT_NONE, FAULT_WATCHDOG, FAULT_IMU };
static const char* faultName(Fault f) {
  switch (f) {
    case FAULT_WATCHDOG: return "WATCHDOG";
    case FAULT_IMU: return "IMU";
    default: return "NONE";
  }
}

static bool armed = false;
static Fault fault = FAULT_NONE;
static BodyVel wanted = {0, 0, 0};   // last VEL command
static BodyVel ramped = {0, 0, 0};   // after the acceleration limit
static unsigned long lastVelMs = 0, lastCycleMs = 0, stillSinceMs = 0, disarmedAtMs = 0;
static uint8_t cycle = 0;
static WheelPi wheelPi[4];
static int32_t countHistory[2][4];   // counts one and two cycles ago
static LineBuffer usbLine;

// ---------------- Output ----------------
static void sendOdo(unsigned long t, const int32_t c[4], int16_t gz) {
  Serial.print(F("ODO "));
  Serial.print(t);
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(' ');
    Serial.print(c[i]);
  }
  Serial.print(' ');
  Serial.println(gz);
}

static void sendRng(unsigned long t) {
  const int16_t* mm = ultrasonicMm();
  Serial.print(F("RNG "));
  Serial.print(t);
  for (uint8_t i = 0; i < US_COUNT; ++i) {
    Serial.print(' ');
    Serial.print(mm[i]);
  }
  Serial.println();
}

static void sendSta(unsigned long t) {
  Serial.print(F("STA "));
  Serial.print(t);
  Serial.print(' ');
  Serial.print(armed ? 1 : 0);
  Serial.print(' ');
  Serial.print(trayDeg());
  Serial.print(' ');
  Serial.print(chargeMillivolts());
  Serial.print(' ');
  Serial.print(batteryMillivolts());
  Serial.print(' ');
  Serial.println(faultName(!imuOk() && fault == FAULT_NONE ? FAULT_IMU : fault));
}

// ---------------- Motion state ----------------
static void haltMotion() {
  wanted = {0, 0, 0};
  ramped = {0, 0, 0};
  for (uint8_t i = 0; i < 4; ++i) wheelPi[i].reset();
  motorsBrake();
}

static void disarm() {
  if (armed) disarmedAtMs = millis();
  armed = false;
  haltMotion();
}

// Bench routine for the feed-forward constants: wheels raised, robot armed. Holds four PWM
// levels for 2 s each and prints each wheel's steady speed. Any serial byte aborts it.
static void feedForwardTest() {
  static const uint8_t LEVELS[] = {60, 90, 120, 150};
  for (uint8_t level = 0; level < sizeof(LEVELS); ++level) {
    const int16_t value = LEVELS[level] > MAX_PWM ? MAX_PWM : LEVELS[level];
    const int16_t pwm[4] = {value, value, value, value};
    motorsWrite(pwm);
    int32_t start[4], end[4];
    unsigned long began = millis();
    while (millis() - began < 1000) {  // let the speed settle
      wdt_reset();
      if (Serial.available()) { disarm(); Serial.println(F("ERR FFTEST_ABORTED")); return; }
    }
    encodersRead(start);
    began = millis();
    while (millis() - began < 1000) {
      wdt_reset();
      if (Serial.available()) { disarm(); Serial.println(F("ERR FFTEST_ABORTED")); return; }
    }
    encodersRead(end);
    Serial.print(F("FF "));
    Serial.print(value);
    for (uint8_t i = 0; i < 4; ++i) {
      Serial.print(' ');
      Serial.print((int)((end[i] - start[i]) * MM_PER_COUNT));  // mm in 1 s = mm/s
    }
    Serial.println();
  }
  disarm();
  Serial.println(F("DONE FFTEST"));
}

// ---------------- Commands ----------------
static void execute(const Command& c, unsigned long now) {
  switch (c.type) {
    case CMD_NONE:
      break;
    case CMD_PING:
      Serial.println(F("PONG"));
      break;
    case CMD_ARM:
      armed = true;
      fault = FAULT_NONE;
      lastVelMs = now;
      haltMotion();
      Serial.println(F("OK ARM"));
      break;
    case CMD_DISARM:
      disarm();
      Serial.println(F("OK DISARM"));
      break;
    case CMD_STOP:
      disarm();
      Serial.println(F("OK STOP"));
      break;
    case CMD_VEL:
      if (armed) {
        wanted.vx = c.a[0];
        wanted.vy = c.a[1];
        wanted.wz = c.a[2] * 0.001f;
        lastVelMs = now;
      }
      break;
    case CMD_TRAY:
      traySetTarget((uint8_t)c.a[0]);
      Serial.println(F("OK TRAY"));
      break;
    case CMD_CAL:
      imuRequestBias();
      Serial.println(F("OK CAL"));
      break;
    case CMD_OLED:
      displayText(c.text);
      break;
    case CMD_FFTEST:
      if (armed) feedForwardTest();
      else Serial.println(F("ERR NOT_ARMED"));
      break;
    default:
      Serial.println(F("ERR UNKNOWN"));
      break;
  }
}

static void readUsb(unsigned long now) {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '!') {  // immediate stop, even in the middle of a damaged line
      disarm();
      Serial.println(F("OK STOP"));
      continue;
    }
    const LineBuffer::Result result = usbLine.feed(c);
    if (result == LineBuffer::LINE) execute(parseLine(usbLine.line()), now);
    else if (result == LineBuffer::OVERFLOW_LINE) Serial.println(F("ERR OVERFLOW"));
  }
}

// Bluetooth accepts only the two demo fallbacks: Z = stop, W = wake.
static void readBluetooth() {
  while (Serial3.available()) {
    const char c = (char)Serial3.read();
    if (c == 'Z') {
      disarm();
      Serial.println(F("EVT BTSTOP"));
    } else if (c == 'W') {
      Serial.println(F("EVT WAKE"));
    }
  }
}

// ---------------- Control cycle ----------------
static void controlCycle(unsigned long now) {
  const float dt = CONTROL_PERIOD_MS * 0.001f;

  int32_t counts[4];
  encodersRead(counts);

  // Wheel speed over the last two cycles: one cycle holds too few counts at low speed.
  float measured[4];
  bool wheelsStill = true;
  for (uint8_t i = 0; i < 4; ++i) {
    const int32_t delta = counts[i] - countHistory[1][i];
    measured[i] = delta * MM_PER_COUNT / (2 * dt);
    if (delta != 0) wheelsStill = false;
    countHistory[1][i] = countHistory[0][i];
    countHistory[0][i] = counts[i];
  }

  if (armed && now - lastVelMs > CMD_WATCHDOG_MS) {
    disarm();
    fault = FAULT_WATCHDOG;
    Serial.println(F("ERR WATCHDOG"));
  }

  if (armed) {
    ramped.vx = stepToward(ramped.vx, wanted.vx, ACCEL_MM_S2 * dt);
    ramped.vy = stepToward(ramped.vy, wanted.vy, ACCEL_MM_S2 * dt);
    ramped.wz = stepToward(ramped.wz, wanted.wz, ACCEL_RAD_S2 * dt);
    float target[4];
    mecanumInverse(ramped, ROTATION_ARM_MM, target);
    limitWheelSpeeds(target, MAX_WHEEL_MM_S);
    int16_t pwm[4];
    for (uint8_t i = 0; i < 4; ++i) {
      pwm[i] = wheelPi[i].update(target[i], measured[i], dt, MAX_PWM, WHEEL_DEAD_MM_S);
    }
    motorsWrite(pwm);
  } else if (now - disarmedAtMs < 1000) {
    motorsBrake();  // stop first...
  } else {
    motorsCoast();  // ...then release, so the robot can be moved by hand
  }

  ultrasonicPingNext();

  const bool commandedStill = !armed || (wanted.vx == 0 && wanted.vy == 0 && wanted.wz == 0);
  if (!(wheelsStill && commandedStill)) stillSinceMs = now;
  imuService(now, now - stillSinceMs > 500);

  sendOdo(now, counts, imuTakeRateMrad());
  if (cycle % RNG_EVERY_CYCLES == 0) sendRng(now);
  if (cycle % STA_EVERY_CYCLES == 0) sendSta(now);
  cycle = (cycle + 1) % (RNG_EVERY_CYCLES * STA_EVERY_CYCLES);

  if (trayService(now)) Serial.println(F("DONE TRAY"));
}

void setup() {
  wdt_disable();
  motorsBegin();  // outputs safe before anything else
  Serial.begin(USB_BAUD);
  Serial3.begin(BLUETOOTH_BAUD);
  encodersBegin();
  ultrasonicBegin();
  imuBegin();
  trayBegin();
  displayBegin();
  for (uint8_t i = 0; i < 4; ++i) {
    wheelPi[i].kp = WHEEL_KP;
    wheelPi[i].ki = WHEEL_KI;
    wheelPi[i].iMax = WHEEL_I_MAX;
    wheelPi[i].ffA = FF_A;
    wheelPi[i].ffB = FF_B;
    wheelPi[i].integral = 0;
  }
  int32_t counts[4];
  encodersRead(counts);
  for (uint8_t i = 0; i < 4; ++i) countHistory[0][i] = countHistory[1][i] = counts[i];
  wdt_enable(WDTO_1S);
  lastCycleMs = millis();
  stillSinceMs = lastCycleMs;
  Serial.println(F("READY NONTOK-SSF 1"));
}

void loop() {
  wdt_reset();
  const unsigned long now = millis();
  readUsb(now);
  readBluetooth();
  if (now - lastCycleMs >= CONTROL_PERIOD_MS) {
    lastCycleMs += CONTROL_PERIOD_MS;
    if (now - lastCycleMs >= CONTROL_PERIOD_MS) lastCycleMs = now;  // fell behind: do not burst
    controlCycle(now);
  }
}
