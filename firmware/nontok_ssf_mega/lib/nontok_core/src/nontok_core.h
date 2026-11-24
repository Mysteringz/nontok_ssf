// Hardware-independent parts of the Nontok SSF firmware: kinematics, wheel control and the
// command parser. No Arduino headers, so the same code is unit-tested on a computer
// (pio test -e native).
#pragma once
#include <stdint.h>

namespace nontok {

// ---------------- Mecanum kinematics ----------------
// Wheel order everywhere: front-left, front-right, rear-left, rear-right.
// Body velocity: vx forward (mm/s), vy left (mm/s), wz counter-clockwise (rad/s).
struct BodyVel {
  float vx, vy, wz;
};

// Wheel rim speeds in mm/s for a body velocity. armMm = half wheelbase + half track.
void mecanumInverse(const BodyVel& body, float armMm, float wheel[4]);

// Scale all four wheels together so the fastest is at most maxMmS (keeps the direction of travel).
void limitWheelSpeeds(float wheel[4], float maxMmS);

// Move `current` toward `target` by at most maxStep.
float stepToward(float current, float target, float maxStep);

// ---------------- Wheel speed controller ----------------
// PWM = feed-forward (ffA + ffB * |target|) + PI on the speed error. The output always has the
// sign of the target: the wheel is never driven backwards to slow down.
struct WheelPi {
  float kp, ki, iMax;  // PWM per mm/s, PWM per mm/s per s, integral clamp in PWM
  float ffA, ffB;      // PWM offset, PWM per mm/s
  float integral;

  void reset() { integral = 0; }
  // target and measured in mm/s (signed), dt in seconds. Targets under deadMmS give 0 (brake).
  int16_t update(float target, float measured, float dt, int16_t maxPwm, float deadMmS);
};

// ---------------- Command parser ----------------
enum CmdType : uint8_t {
  CMD_NONE = 0,  // empty line
  CMD_PING,
  CMD_ARM,
  CMD_DISARM,
  CMD_VEL,     // a[0..2] = vx mm/s, vy mm/s, wz mrad/s
  CMD_STOP,
  CMD_TRAY,    // a[0] = degrees
  CMD_CAL,
  CMD_OLED,    // text = rest of the line
  CMD_FFTEST,  // a[0..2] = up to 3 PWM levels, 1..FFTEST_MAX_PWM; unused slots 0 (none = defaults)
  CMD_INVALID  // unknown word, wrong argument count or out-of-range value
};

struct Command {
  CmdType type;
  long a[3];
  const char* text;
};

const long VEL_LIMIT_MM_S = 500;
const long VEL_LIMIT_MRAD_S = 3000;
const long TRAY_LIMIT_DEG = 130;  // stepper travel; must equal TRAY_MAX_DEG in config.h
const long FFTEST_MAX_PWM = 120;  // must equal MAX_PWM in config.h

// Parses one line without its newline. The buffer is modified (split into words).
Command parseLine(char* line);

// Collects serial bytes into lines. Lines longer than the buffer are dropped whole.
class LineBuffer {
 public:
  static const uint8_t CAPACITY = 48;
  enum Result : uint8_t { PENDING, LINE, OVERFLOW_LINE };

  // Feed one byte. LINE: line() holds a complete line. OVERFLOW_LINE: an over-long line just ended.
  Result feed(char c);
  char* line() { return buf_; }

 private:
  char buf_[CAPACITY];
  uint8_t length_ = 0;
  bool overflow_ = false;
};

}  // namespace nontok
