// Quadrature wheel encoders. Channel A triggers an interrupt on every edge; channel B, read in
// the handler, gives the direction. Ports are read directly to keep the handlers short.
//   A: D18 = PD3, D19 = PD2, D3 = PE5, D2 = PE4      B: D31 = PC6, D38 = PD7, D49 = PL0, A1 = PF1
#include "config.h"
#include "drivers.h"

static volatile int32_t count[4] = {0, 0, 0, 0};
static volatile int8_t commandDir[4] = {1, 1, 1, 1};

// A and B differ after an edge in one direction of rotation and match in the other.
static inline void tick(uint8_t wheel, bool a, bool b) {
  count[wheel] += (a != b) ? ENC_SIGN[wheel] : -ENC_SIGN[wheel];
}

static void isr0() { tick(0, PIND & _BV(PD3), PINC & _BV(PC6)); }
static void isr1() { tick(1, PIND & _BV(PD2), PIND & _BV(PD7)); }
static void isr2() { tick(2, PINE & _BV(PE5), PINL & _BV(PL0)); }
static void isr3() {
#if M4_DIR_FROM_COMMAND
  count[3] += commandDir[3];
#else
  tick(3, PINE & _BV(PE4), PINF & _BV(PF1));
#endif
}

void encodersBegin() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(ENC_A_PIN[i], INPUT);
    pinMode(ENC_B_PIN[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[0]), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[1]), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[2]), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[3]), isr3, CHANGE);
}

void encodersRead(int32_t counts[4]) {
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) counts[i] = count[i];
  interrupts();
}

void encodersSetCommandDir(uint8_t wheel, int8_t dir) {
  if (dir != 0) commandDir[wheel] = dir > 0 ? 1 : -1;  // keep the last direction while coasting
}
