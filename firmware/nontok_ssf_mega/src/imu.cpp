// MPU6050 yaw-rate gyro. The chip samples at 200 Hz into its FIFO and every sample is used.
// Register setup and the I2C safeguards follow ../Nontok (hardware_probe_mega, mpu_gyro.h):
// motor noise on SDA/SCL once hung the bus, so every transfer has a 25 ms timeout and a
// stuck bus is clocked free.
#include <Wire.h>

#include "config.h"
#include "drivers.h"

static const uint8_t REG_SMPLRT_DIV = 0x19, REG_CONFIG = 0x1a, REG_GYRO_CONFIG = 0x1b;
static const uint8_t REG_FIFO_EN = 0x23, REG_INT_ENABLE = 0x38, REG_USER_CTRL = 0x6a;
static const uint8_t REG_PWR_MGMT_1 = 0x6b, REG_FIFO_COUNT = 0x72, REG_FIFO_RW = 0x74;
static const uint8_t REG_WHO_AM_I = 0x75;
static const float LSB_PER_DPS = 65.5f;        // +/-500 deg/s range
static const float MRAD_PER_DEG = 17.4532925f;
static const uint16_t BIAS_SAMPLES = 200;      // 1 s
static const uint16_t FIFO_OVERFLOW_BYTES = 512;

static bool present = false, biasValid = false, biasWanted = true;
static float bias = 0;
static long biasSum = 0;
static uint16_t biasCount = 0;
static long rateSum = 0;
static uint16_t rateCount = 0;
static int16_t lastRate = 0;
static uint16_t timeouts = 0;
static unsigned long lastHealthMs = 0;

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

static bool failed() {
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    ++timeouts;
    busRecover();
  }
  present = false;
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

static bool fifoRestart() {
  return writeReg(REG_USER_CTRL, 0x04) && writeReg(REG_USER_CTRL, 0x40);  // reset, then enable
}

static bool setupDevice() {
  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) return false;
  // MPU6050 and register-compatible parts.
  if (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x72 && who != 0x73) return false;
  const bool ok = writeReg(REG_PWR_MGMT_1, 0x01) &&  // wake, gyro PLL clock
                  writeReg(REG_SMPLRT_DIV, 4) &&     // 1 kHz / 5 = 200 Hz
                  writeReg(REG_CONFIG, 0x03) &&      // ~44 Hz low-pass
                  writeReg(REG_GYRO_CONFIG, 0x08) && // +/-500 deg/s
                  writeReg(REG_INT_ENABLE, 0x00) &&
                  writeReg(REG_FIFO_EN, 0x10) &&     // yaw (Z) gyro only
                  fifoRestart();
  present = ok;
  return ok;
}

static void drainFifo(bool still) {
  uint8_t head[2];
  if (!readRegs(REG_FIFO_COUNT, head, 2)) return;
  uint16_t bytes = ((uint16_t)head[0] << 8) | head[1];
  if (bytes >= FIFO_OVERFLOW_BYTES) {  // fell behind: samples were lost, start clean
    fifoRestart();
    return;
  }
  bytes &= ~1u;  // whole samples only
  while (bytes > 0) {
    uint8_t chunk = bytes > 30 ? 30 : (uint8_t)bytes;  // Wire buffer is 32 bytes
    uint8_t data[30];
    if (!readRegs(REG_FIFO_RW, data, chunk)) return;
    for (uint8_t i = 0; i < chunk; i += 2) {
      const int16_t raw = (int16_t)(((uint16_t)data[i] << 8) | data[i + 1]);
      rateSum += raw;
      ++rateCount;
      if (biasWanted && still) {
        biasSum += raw;
        if (++biasCount >= BIAS_SAMPLES) {
          bias = (float)biasSum / biasCount;
          biasValid = true;
          biasWanted = false;
        }
      }
    }
    bytes -= chunk;
  }
  if (biasWanted && !still) {  // moved during the window: start again
    biasSum = 0;
    biasCount = 0;
  }
}

void imuBegin() {
  busBegin();
  setupDevice();
}

void imuService(unsigned long nowMs, bool still) {
  if (nowMs - lastHealthMs >= 1000) {
    lastHealthMs = nowMs;
    uint8_t who = 0;
    if (!present || !readRegs(REG_WHO_AM_I, &who, 1)) {
      setupDevice();  // absent or scrambled after a brown-out: configure again
      return;
    }
  }
  if (present) drainFifo(still);
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

void imuRequestBias() {
  biasWanted = true;
  biasSum = 0;
  biasCount = 0;
}

uint16_t imuTimeouts() { return timeouts; }
