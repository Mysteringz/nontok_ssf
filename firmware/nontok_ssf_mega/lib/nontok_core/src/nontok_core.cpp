#include "nontok_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace nontok {

void mecanumInverse(const BodyVel& body, float armMm, float wheel[4]) {
  const float w = body.wz * armMm;
  wheel[0] = body.vx - body.vy - w;  // front-left
  wheel[1] = body.vx + body.vy + w;  // front-right
  wheel[2] = body.vx + body.vy - w;  // rear-left
  wheel[3] = body.vx - body.vy + w;  // rear-right
}

void limitWheelSpeeds(float wheel[4], float maxMmS) {
  float peak = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    const float mag = fabsf(wheel[i]);
    if (mag > peak) peak = mag;
  }
  if (peak <= maxMmS || peak <= 0) return;
  const float scale = maxMmS / peak;
  for (uint8_t i = 0; i < 4; ++i) wheel[i] *= scale;
}

float stepToward(float current, float target, float maxStep) {
  const float delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

int16_t WheelPi::update(float target, float measured, float dt, int16_t maxPwm, float deadMmS) {
  const float magnitude = fabsf(target);
  if (magnitude < deadMmS) {
    integral = 0;
    return 0;
  }
  const float sign = target > 0 ? 1.0f : -1.0f;
  const float error = magnitude - sign * measured;  // speed still missing along the target direction
  integral += ki * error * dt;
  if (integral > iMax) integral = iMax;
  if (integral < -iMax) integral = -iMax;
  float out = ffA + ffB * magnitude + kp * error + integral;
  if (out < 0) out = 0;
  if (out > maxPwm) out = maxPwm;
  return (int16_t)(sign * out);
}

// Splits off the next space-separated word; returns 0 at the end of the line.
static char* nextWord(char** cursor) {
  char* p = *cursor;
  while (*p == ' ') ++p;
  if (*p == '\0') {
    *cursor = p;
    return 0;
  }
  char* word = p;
  while (*p != '\0' && *p != ' ') ++p;
  if (*p == ' ') *p++ = '\0';
  *cursor = p;
  return word;
}

static bool parseLong(const char* word, long* value) {
  if (word == 0 || *word == '\0') return false;
  char* end = 0;
  *value = strtol(word, &end, 10);
  return *end == '\0';
}

static Command make(CmdType type) {
  Command c;
  c.type = type;
  c.a[0] = c.a[1] = c.a[2] = 0;
  c.text = 0;
  return c;
}

Command parseLine(char* line) {
  // Strip a trailing carriage return and spaces.
  size_t length = strlen(line);
  while (length > 0 && (line[length - 1] == '\r' || line[length - 1] == ' ')) line[--length] = '\0';

  char* cursor = line;
  const char* word = nextWord(&cursor);
  if (word == 0) return make(CMD_NONE);

  if (strcmp(word, "OLED") == 0) {
    Command c = make(CMD_OLED);
    while (*cursor == ' ') ++cursor;
    c.text = cursor;
    return c;
  }

  CmdType type = CMD_INVALID;
  uint8_t wanted = 0;
  if (strcmp(word, "PING") == 0) type = CMD_PING;
  else if (strcmp(word, "ARM") == 0) type = CMD_ARM;
  else if (strcmp(word, "DISARM") == 0) type = CMD_DISARM;
  else if (strcmp(word, "STOP") == 0) type = CMD_STOP;
  else if (strcmp(word, "CAL") == 0) type = CMD_CAL;
  else if (strcmp(word, "FFTEST") == 0) type = CMD_FFTEST;
  else if (strcmp(word, "VEL") == 0) { type = CMD_VEL; wanted = 3; }
  else if (strcmp(word, "TRAY") == 0) { type = CMD_TRAY; wanted = 1; }
  if (type == CMD_INVALID) return make(CMD_INVALID);

  Command c = make(type);
  for (uint8_t i = 0; i < wanted; ++i) {
    if (!parseLong(nextWord(&cursor), &c.a[i])) return make(CMD_INVALID);
  }
  if (nextWord(&cursor) != 0) return make(CMD_INVALID);  // extra arguments

  if (type == CMD_VEL) {
    if (labs(c.a[0]) > VEL_LIMIT_MM_S || labs(c.a[1]) > VEL_LIMIT_MM_S ||
        labs(c.a[2]) > VEL_LIMIT_MRAD_S) return make(CMD_INVALID);
  }
  if (type == CMD_TRAY && (c.a[0] < 0 || c.a[0] > TRAY_LIMIT_DEG)) return make(CMD_INVALID);
  return c;
}

LineBuffer::Result LineBuffer::feed(char c) {
  if (c == '\n') {
    const bool wasOverflow = overflow_;
    buf_[length_] = '\0';
    length_ = 0;
    overflow_ = false;
    return wasOverflow ? OVERFLOW_LINE : LINE;
  }
  if (overflow_) return PENDING;
  if (length_ >= CAPACITY - 1) {
    overflow_ = true;
    length_ = 0;
    return PENDING;
  }
  buf_[length_++] = c;
  return PENDING;
}

}  // namespace nontok
