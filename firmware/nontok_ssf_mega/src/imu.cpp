// MPU6050 yaw-rate gyro. The chip samples at 200 Hz into its FIFO and every sample is used.
// Zeroing, FIFO handling and recovery follow ../Nontok (hardware_probe_mega gyroBegin / gyroZero /
// gyroDrain / gyroConfigure, mpu_gyro.h), which proved better on the robot (24 Nov: this file's
// earlier version let a failed FIFO read misalign the samples, and the robot spun on a false
// +190 deg turn):
//  - every transfer has a 25 ms timeout; a stuck bus is clocked free (motor and stepper noise)
//  - boot: up to 3 attempts, each with a full device reset, a bus recovery between them
//  - bias from a 2 s window (400 samples), applied only if the robot was still: noise SD under
//    STILL_SD raw. A window with movement is thrown away and the next one tried, so the robot
//    zeroes itself as soon as it stands still
//  - a failed FIFO read restarts the FIFO: the bytes left behind would pair the high byte of one
//    sample with the low byte of the next
//  - FIFO overflow detected by the chip's flag and by the byte count (1000 of the MPU6050's 1024
//    bytes, 2.5 s; the other parts have 512): the FIFO restarts, the lost samples are not guessed
//  - saturated samples are dropped
//  - no sample for 300 ms: setup applied again (a reset or a supply dip puts the chip to sleep);
//    the bias is kept
#include <Wire.h>

#include "config.h"
#include "drivers.h"

static const uint8_t REG_SMPLRT_DIV = 0x19, REG_CONFIG = 0x1a, REG_GYRO_CONFIG = 0x1b;
static const uint8_t REG_FIFO_EN = 0x23, REG_INT_PIN_CFG = 0x37, REG_INT_ENABLE = 0x38;
static const uint8_t REG_INT_STATUS = 0x3a, REG_USER_CTRL = 0x6a, REG_PWR_MGMT_1 = 0x6b;
static const uint8_t REG_FIFO_COUNT = 0x72, REG_FIFO_RW = 0x74, REG_WHO_AM_I = 0x75;
static const uint8_t INT_FIFO_OVERFLOW = 0x10, INT_DATA_READY = 0x01;
static const uint8_t USER_FIFO_ON = 0x40, USER_FIFO_RESET = 0x04;
static const float LSB_PER_DPS = 65.5f;        // +/-500 deg/s range
static const float MRAD_PER_DEG = 17.4532925f;
static const uint16_t ZERO_SAMPLES = 400;      // 2 s bias window
static const float STILL_SD = 25.0f;           // raw; normally 2-5 when still
static const int16_t SATURATION = 32700;
static const unsigned long STUCK_MS = 300;     // 200 Hz: 300 ms with no sample = stuck

static bool present = false, biasValid = false, biasWanted = true;
static uint8_t who = 0;
static float bias = 0;
static float zeroSum = 0, zeroSquares = 0;
static uint16_t zeroCount = 0;
static long rateSum = 0;
static uint16_t rateCount = 0;
static int16_t lastRate = 0;
static uint16_t timeouts = 0;
static unsigned long lastSampleMs = 0;

static void busBegin() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
}

// Clock a slave that is holding SDA low until it lets go, then send STOP.
static void busRecover() {
  Wire.end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; ++i) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  busBegin();
}

// After any failed transfer: only a timeout means the bus itself was stuck. The chip is not
// marked absent for one failed transfer (stepper noise); the stuck rule configures it again.
static bool failed() {
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    ++timeouts;
    busRecover();
  }
  return false;
}

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0 || failed();
}

static bool readRegs(uint8_t reg, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return failed();
  if (Wire.requestFrom(IMU_ADDRESS, length) != length) return failed();
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

static void fifoRestart() {
  writeReg(REG_USER_CTRL, USER_FIFO_RESET);
  writeReg(REG_USER_CTRL, USER_FIFO_ON);
}

static void zeroClear() {
  zeroSum = zeroSquares = 0;
  zeroCount = 0;
}

// Wake on the gyro PLL, 200 Hz samples, ~44 Hz low-pass, +/-500 deg/s, yaw (Z) only into the
// FIFO (2 bytes a sample: the MPU6050's 1024 bytes hold 2.5 s). INT latched, cleared by any read.
static bool applySetup() {
  bool ok = writeReg(REG_PWR_MGMT_1, 0x01);
  ok = writeReg(REG_SMPLRT_DIV, 4) && ok;
  ok = writeReg(REG_CONFIG, 0x03) && ok;
  ok = writeReg(REG_GYRO_CONFIG, 0x08) && ok;
  ok = writeReg(REG_INT_PIN_CFG, 0x30) && ok;
  ok = writeReg(REG_INT_ENABLE, INT_FIFO_OVERFLOW | INT_DATA_READY) && ok;
  ok = writeReg(REG_FIFO_EN, 0) && ok;
  ok = writeReg(REG_USER_CTRL, USER_FIFO_RESET) && ok;
  ok = writeReg(REG_FIFO_EN, 0x10) && ok;
  ok = writeReg(REG_USER_CTRL, USER_FIFO_ON) && ok;
  return ok;
}

static bool knownPart(uint8_t id) {
  // MPU6050 and register-compatible parts (as mpuModel in ../Nontok).
  return id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72 || id == 0x73 || id == 0x12 ||
         id == 0x98 || id == 0xaf;
}

// A sample window that could not be judged (samples lost) is started again.
static void zeroAbandon() { zeroClear(); }

// The bias is taken only from a still robot: a zero taken while the robot moved gave a 30 deg/s
// yaw error in ../Nontok. Applied only if the noise over the whole window stayed small.
static void zeroFinish() {
  const float mean = zeroSum / zeroCount;
  const float var = zeroSquares / zeroCount - mean * mean;
  const float sd = var > 0 ? sqrt(var) : 0;
  if (sd < STILL_SD) {
    bias = mean;
    biasValid = true;
    biasWanted = false;
  }
  zeroClear();
}

static void drainFifo(bool still) {
  uint8_t status, head[2];
  if (!readRegs(REG_INT_STATUS, &status, 1) || !readRegs(REG_FIFO_COUNT, head, 2)) return;
  uint16_t bytes = ((uint16_t)head[0] << 8) | head[1];
  const uint16_t limit = who == 0x68 ? 1000 : 512;
  if ((status & INT_FIFO_OVERFLOW) || bytes >= limit) {  // samples were lost: start clean
    fifoRestart();
    zeroAbandon();
    return;
  }
  bytes &= ~1u;  // whole samples only
  const bool fresh = bytes > 0;
  while (bytes > 0) {
    uint8_t chunk = bytes > 30 ? 30 : (uint8_t)bytes;  // Wire buffer is 32 bytes
    uint8_t data[30];
    if (!readRegs(REG_FIFO_RW, data, chunk)) {
      // Part of the chunk may have left the FIFO: what remains is no longer sample-aligned.
      fifoRestart();
      zeroAbandon();
      return;
    }
    for (uint8_t i = 0; i < chunk; i += 2) {
      const int16_t raw = (int16_t)(((uint16_t)data[i] << 8) | data[i + 1]);
      if (raw >= SATURATION || raw <= -SATURATION) continue;
      rateSum += raw;
      ++rateCount;
      if (biasWanted) {
        if (still) {
          zeroSum += raw;
          zeroSquares += (float)raw * raw;
          if (++zeroCount >= ZERO_SAMPLES) zeroFinish();
        } else {
          zeroClear();  // moved during the window: the next still window is tried
        }
      }
    }
    bytes -= chunk;
  }
  if (fresh) lastSampleMs = millis();
}

void imuBegin() {
  busBegin();
  present = biasValid = false;
  biasWanted = true;
  zeroClear();
  // The MPU keeps power through a Mega reset, so a chip left mid-transfer can fail setup: retry
  // after a bus recovery, each time with a full device reset.
  for (uint8_t attempt = 0; attempt < 3 && !present; ++attempt) {
    if (attempt) busRecover();
    who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1) || !knownPart(who)) continue;
    const bool reset = writeReg(REG_PWR_MGMT_1, 0x80);  // volatile device reset
    delay(100);
    present = applySetup() && reset;
  }
  if (!present) return;
  delay(200);  // gyro start-up and low-pass settling before the first bias window
  fifoRestart();
  lastSampleMs = millis();
}

void imuService(unsigned long nowMs, bool still) {
  if (!present) {
    // Absent at boot or since: look for it once a second.
    static unsigned long lastTryMs = 0;
    if (nowMs - lastTryMs < 1000) return;
    lastTryMs = nowMs;
    who = 0;
    if (readRegs(REG_WHO_AM_I, &who, 1) && knownPart(who) && applySetup()) {
      present = true;
      lastSampleMs = nowMs;
    }
    return;
  }
  if (nowMs - lastSampleMs > STUCK_MS) {
    // No samples: reset or asleep after a supply dip. Set up again, keeping the bias.
    lastSampleMs = nowMs;
    zeroAbandon();
    if (!applySetup()) {
      uint8_t id = 0;
      if (!readRegs(REG_WHO_AM_I, &id, 1)) present = false;  // gone: look for it again
    }
    return;
  }
  drainFifo(still);
}

int16_t imuTakeRateMrad() {
  if (rateCount > 0) {
    const float meanRaw = (float)rateSum / rateCount;
    const float dps = (meanRaw - (biasValid ? bias : 0)) / LSB_PER_DPS;
    lastRate = (int16_t)(GYRO_SIGN * dps * MRAD_PER_DEG);
    rateSum = 0;
    rateCount = 0;
  } else if (!present) {
    lastRate = 0;
  }
  return lastRate;
}

bool imuOk() { return present && biasValid; }

// CAL: measure the bias again the next time the robot is still (2 s window, still check).
// The old bias stays in use until a new one is accepted.
void imuRequestBias() {
  zeroClear();
  biasWanted = true;
}

uint16_t imuTimeouts() { return timeouts; }
