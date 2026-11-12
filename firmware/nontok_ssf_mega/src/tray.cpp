// Tray servo, voltage sensing and the optional status display.
#include <Servo.h>

#include "config.h"
#include "drivers.h"

#if OLED_ENABLED
#include <U8x8lib.h>
static U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(U8X8_PIN_NONE);
#endif

// ---------------- Tray ----------------
static Servo servo;
static uint8_t currentDeg = 0, targetDeg = 0;
static bool moving = false;
static unsigned long restSinceMs = 0;

static void writeServo() {
  servo.write(TRAY_REST_SERVO_DEG + TRAY_SERVO_DIR * (int)currentDeg);
}

void trayBegin() {
  // Attached only on the first command, so a robot without a tray never drives the pin.
}

void traySetTarget(uint8_t deg) {
  targetDeg = deg > TRAY_MAX_DEG ? TRAY_MAX_DEG : deg;
  moving = true;
  if (!servo.attached()) {
    servo.attach(TRAY_SERVO_PIN);
    writeServo();
  }
}

bool trayService(unsigned long nowMs) {
  if (moving) {
    if (currentDeg < targetDeg) {
      currentDeg = (targetDeg - currentDeg > TRAY_UP_DEG_PER_CYCLE) ? currentDeg + TRAY_UP_DEG_PER_CYCLE : targetDeg;
    } else if (currentDeg > targetDeg) {
      currentDeg = (currentDeg - targetDeg > TRAY_DOWN_DEG_PER_CYCLE) ? currentDeg - TRAY_DOWN_DEG_PER_CYCLE : targetDeg;
    }
    writeServo();
    if (currentDeg == targetDeg) {
      moving = false;
      restSinceMs = nowMs;
      return true;
    }
    return false;
  }
  // Flat and idle: release the servo so it neither buzzes nor loads the supply.
  if (currentDeg == 0 && servo.attached() && nowMs - restSinceMs >= TRAY_DETACH_MS) servo.detach();
  return false;
}

uint8_t trayDeg() { return currentDeg; }

// ---------------- Voltage sensing ----------------
static uint16_t readMillivolts(uint8_t pin, float divider) {
  return (uint16_t)(analogRead(pin) * (5000.0f / 1023.0f) * divider);
}

uint16_t chargeMillivolts() {
#if CHARGE_SENSE_FITTED
  return readMillivolts(CHARGE_SENSE_PIN, CHARGE_DIVIDER);
#else
  (void)readMillivolts;
  return 0;
#endif
}

uint16_t batteryMillivolts() {
#if BATTERY_SENSE_FITTED
  return readMillivolts(BATTERY_SENSE_PIN, BATTERY_DIVIDER);
#else
  return 0;
#endif
}

// ---------------- Display ----------------
void displayBegin() {
#if OLED_ENABLED
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.drawString(0, 0, "NONTOK SSF");
#endif
}

void displayText(const char* text) {
#if OLED_ENABLED
  char line[17];
  uint8_t i = 0;
  for (; i < 16 && text[i] != '\0'; ++i) line[i] = text[i];
  for (; i < 16; ++i) line[i] = ' ';  // overwrite what was there
  line[16] = '\0';
  oled.drawString(0, 2, line);
#else
  (void)text;
#endif
}
