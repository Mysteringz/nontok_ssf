// Nontok SSF: Mega firmware. Wheel speed control, sensors and safety; the Jetson decides
// where to go. Protocol: PLAN.md section 6.3.
#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "drivers.h"
#include "nontok_core.h"

using namespace nontok;

// The parser (hardware-free, no config.h) and the tray code must allow the same travel.
static_assert(TRAY_LIMIT_DEG == TRAY_MAX_DEG, "TRAY_LIMIT_DEG must equal TRAY_MAX_DEG");

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

// Bench routine for the feed-forward constants: wheels raised, robot armed. Open loop: all four
// wheels get the same PWM, forward at each level, then reverse, 1 s to settle and 1 s measured
// per level. Prints "FF <signed pwm> <mm/s x4>" per level (wheel-forward positive). M4 has no
// channel B, so its sign follows the command. Any serial byte aborts it.
static bool waitOrAbort(unsigned long ms) {
  const unsigned long began = millis();
  while (millis() - began < ms) {
    wdt_reset();
    if (Serial.available()) return false;
  }
  return true;
}

static void feedForwardTest(const Command& c) {
  // "FFTEST a b c": up to 3 levels (parser allows 1..FFTEST_MAX_PWM); plain "FFTEST": 60 90 120.
  static_assert(FFTEST_MAX_PWM == MAX_PWM, "FFTEST_MAX_PWM must equal MAX_PWM");
  uint8_t levels[3] = {60, 90, 120};
  uint8_t count = 3;
  if (c.a[0] != 0) {
    count = 0;
    while (count < 3 && c.a[count] != 0) { levels[count] = (uint8_t)c.a[count]; ++count; }
  }
  for (int8_t dir = 1; dir >= -1; dir -= 2) {
    for (uint8_t level = 0; level < count; ++level) {
      const int16_t value = dir * (int16_t)levels[level];
      const int16_t pwm[4] = {value, value, value, value};
      motorsWrite(pwm);
      int32_t start[4], end[4];
      if (!waitOrAbort(1000)) goto aborted;  // let the speed settle
      encodersRead(start);
      const unsigned long t0 = millis();
      if (!waitOrAbort(1000)) goto aborted;
      encodersRead(end);
      const float seconds = (millis() - t0) * 0.001f;
      Serial.print(F("FF "));
      Serial.print(value);
      for (uint8_t i = 0; i < 4; ++i) {
        Serial.print(' ');
        Serial.print((int)((end[i] - start[i]) * MM_PER_COUNT / seconds));
      }
      Serial.println();
    }
    motorsBrake();  // stop before reversing
    if (!waitOrAbort(dir > 0 ? 700 : 0)) goto aborted;
  }
  disarm();
  Serial.println(F("DONE FFTEST"));
  return;
aborted:
  disarm();
  Serial.println(F("ERR FFTEST_ABORTED"));
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
      trayStop();
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
      traySetTarget((int16_t)constrain(c.a[0], 0L, (long)TRAY_MAX_DEG));
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
      if (!armed) Serial.println(F("ERR NOT_ARMED"));
      else if (trayMoving()) Serial.println(F("ERR TRAY_BUSY"));  // FFTEST blocks the stepper
      else feedForwardTest(c);
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
      trayStop();
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
      trayStop();
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

  // While the tray stepper moves, nothing here may block for more than a step period: a
  // stall at cruise speed can make the motor skip. So no ultrasonic ping (blocks up to 15 ms;
  // RNG repeats the last readings, the robot is stationary) and one line per cycle, which fits
  // the 64-byte serial buffer, instead of up to three.
  const bool trayBusy = trayMoving();
  if (!trayBusy) ultrasonicPingNext();

  const bool commandedStill = !armed || (wanted.vx == 0 && wanted.vy == 0 && wanted.wz == 0);
  if (!(wheelsStill && commandedStill)) stillSinceMs = now;
  imuService(now, now - stillSinceMs > 500);

  const bool rng = cycle % RNG_EVERY_CYCLES == 0, sta = cycle % STA_EVERY_CYCLES == 0;
  if (!trayBusy) {
    sendOdo(now, counts, imuTakeRateMrad());
    if (rng) sendRng(now);
    if (sta) sendSta(now);
  } else if (sta) {
    sendSta(now);
  } else if (rng) {
    sendRng(now);
  } else {
    sendOdo(now, counts, imuTakeRateMrad());  // the rate is the mean since the last ODO
  }
  cycle = (cycle + 1) % (RNG_EVERY_CYCLES * STA_EVERY_CYCLES);
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
    wheelPi[i].ffA = FF_A[i];
    wheelPi[i].ffB = FF_B[i];
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
  if (trayService(now)) Serial.println(F("DONE TRAY"));  // every pass: steps are timed in us
  if (now - lastCycleMs >= CONTROL_PERIOD_MS) {
    lastCycleMs += CONTROL_PERIOD_MS;
    if (now - lastCycleMs >= CONTROL_PERIOD_MS) lastCycleMs = now;  // fell behind: do not burst
    controlCycle(now);
  }
}
