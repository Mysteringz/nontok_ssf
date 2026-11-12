// Five HC-SR04-type sensors, one ping per control cycle so their echoes cannot mix.
#include "config.h"
#include "drivers.h"

static int16_t rangeMm[US_COUNT] = {-1, -1, -1, -1, -1};
static uint8_t turn = 0;

void ultrasonicBegin() {
  for (uint8_t i = 0; i < US_COUNT; ++i) {
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_TRIG[i], OUTPUT);
    pinMode(US_ECHO[i], INPUT);
  }
}

void ultrasonicPingNext() {
  const uint8_t sensor = US_PING_ORDER[turn];
  turn = (turn + 1) % US_COUNT;

  digitalWrite(US_TRIG[sensor], LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG[sensor], HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG[sensor], LOW);

  // Sound travels out and back at 343 m/s: 0.1715 mm per microsecond of echo.
  const unsigned long echoUs = pulseIn(US_ECHO[sensor], HIGH, US_TIMEOUT_US);
  const int16_t mm = (int16_t)(echoUs * 0.1715f);
  // No echo is "unknown", never "far".
  rangeMm[sensor] = (echoUs == 0 || mm < US_MIN_MM) ? -1 : mm;
}

const int16_t* ultrasonicMm() { return rangeMm; }
