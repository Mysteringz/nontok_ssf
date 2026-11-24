// Tray dump stepper, voltage sensing and the optional status display.
#include "config.h"
#include "drivers.h"
#include "stepper_driver.h"

#if OLED_ENABLED
#include <U8x8lib.h>
static U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(U8X8_PIN_NONE);
#endif

// ---------------- Tray ----------------
// The driver holds torque from the first move up until the tray is back at 0 deg, so a tipped
// tray cannot fall under the item's weight; at rest it is released after TRAY_RELEASE_MS.
static StepperDriver stepper;
static bool moving = false, releasePending = false;
static unsigned long restSinceMs = 0;

void trayBegin() {
  StepperDriver::Config c;
  c.periodUs = TRAY_STEP_PERIOD_US;
  c.startPeriodUs = TRAY_START_PERIOD_US;
  c.rampSteps = TRAY_RAMP_STEPS;
  c.invert = TRAY_STEPPER_INVERT;
  stepper = StepperDriver(c);
  stepper.begin();  // driver off, position 0
}

void traySetTarget(int16_t deg) {
  if (deg < 0) deg = 0;
  if (deg > TRAY_MAX_DEG) deg = TRAY_MAX_DEG;
  stepper.setHold(true);
  releasePending = false;
  stepper.startSteps(stepper.degreesToSteps(deg) - stepper.position());
  moving = true;  // a zero-length move reports DONE on the next pass
}

void trayStop() {
  stepper.stop();  // hold stays on: a tipped tray keeps its place
  moving = false;
}

bool trayService(unsigned long nowMs) {
  if (moving) {
    if (stepper.service()) return false;
    moving = false;
    if (stepper.position() == 0) {
      releasePending = true;
      restSinceMs = nowMs;
    }
    return true;
  }
  if (releasePending && nowMs - restSinceMs >= TRAY_RELEASE_MS) {
    releasePending = false;
    stepper.setHold(false);  // flat and idle: driver off, no heat, no supply load
  }
  return false;
}

bool trayMoving() { return moving; }

uint8_t trayDeg() {
  const float d = stepper.positionDegrees();
  return d <= 0 ? 0 : d >= 255 ? 255 : (uint8_t)(d + 0.5f);
}

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
