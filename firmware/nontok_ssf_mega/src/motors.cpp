// TB6612 outputs. Direction convention follows ../Nontok core firmware: after applying
// MOTOR_SIGN, positive drives IN1 HIGH / IN2 LOW.
#include "config.h"
#include "drivers.h"

void motorsBegin() {
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(MOTOR_PWM[i], LOW);
    pinMode(MOTOR_PWM[i], OUTPUT);
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
  }
  motorsBrake();
}

void motorsWrite(const int16_t pwm[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    int16_t value = pwm[i];
    encodersSetCommandDir(i, value > 0 ? 1 : (value < 0 ? -1 : 0));
    if (value == 0) {
      // Both inputs HIGH with PWM HIGH is the TB6612 short brake.
      digitalWrite(MOTOR_IN1[i], HIGH);
      digitalWrite(MOTOR_IN2[i], HIGH);
      analogWrite(MOTOR_PWM[i], 255);
      continue;
    }
    const bool forward = (value > 0) == (MOTOR_SIGN[i] > 0);
    int16_t magnitude = abs(value);
    if (magnitude > MAX_PWM) magnitude = MAX_PWM;
    digitalWrite(MOTOR_IN1[i], forward ? HIGH : LOW);
    digitalWrite(MOTOR_IN2[i], forward ? LOW : HIGH);
    analogWrite(MOTOR_PWM[i], magnitude);
  }
}

void motorsBrake() {
  const int16_t zero[4] = {0, 0, 0, 0};
  motorsWrite(zero);
}

void motorsCoast() {
  for (uint8_t i = 0; i < 4; ++i) {
    analogWrite(MOTOR_PWM[i], 0);
    digitalWrite(MOTOR_IN1[i], LOW);
    digitalWrite(MOTOR_IN2[i], LOW);
  }
}
