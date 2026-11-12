// Host tests for lib/nontok_core. Run with: pio test -e native
#include <string.h>
#include <unity.h>

#include "nontok_core.h"

using namespace nontok;

void setUp() {}
void tearDown() {}

static Command parse(const char* text) {
  static char buffer[64];
  strncpy(buffer, text, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  return parseLine(buffer);
}

void test_inverse_forward() {
  float w[4];
  mecanumInverse({200, 0, 0}, 160, w);
  for (int i = 0; i < 4; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-3, 200, w[i]);
}

void test_inverse_strafe_left() {
  float w[4];
  mecanumInverse({0, 100, 0}, 160, w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -100, w[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, w[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, w[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -100, w[3]);
}

void test_inverse_rotate_ccw() {
  float w[4];
  mecanumInverse({0, 0, 1.0f}, 160, w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -160, w[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 160, w[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -160, w[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 160, w[3]);
}

void test_limit_keeps_ratio() {
  float w[4] = {800, 400, -400, 200};
  limitWheelSpeeds(w, 400);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 400, w[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 200, w[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -200, w[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, w[3]);
}

void test_step_toward() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 12, stepToward(0, 100, 12));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, -12, stepToward(0, -100, 12));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 5, stepToward(0, 5, 12));
}

static WheelPi makePi() {
  WheelPi pi;
  pi.kp = 0.15f;
  pi.ki = 0.6f;
  pi.iMax = 40;
  pi.ffA = 50;
  pi.ffB = 0.35f;
  pi.integral = 0;
  return pi;
}

void test_pi_zero_target_brakes_and_clears() {
  WheelPi pi = makePi();
  pi.integral = 20;
  TEST_ASSERT_EQUAL_INT16(0, pi.update(5, 100, 0.04f, 120, 15));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, pi.integral);
}

void test_pi_feed_forward_at_speed() {
  WheelPi pi = makePi();
  // On target: output is the feed-forward alone.
  TEST_ASSERT_EQUAL_INT16(85, pi.update(100, 100, 0.04f, 255, 15));
  TEST_ASSERT_EQUAL_INT16(-85, pi.update(-100, -100, 0.04f, 255, 15));
}

void test_pi_pushes_harder_when_slow_and_respects_cap() {
  WheelPi pi = makePi();
  const int16_t slow = pi.update(100, 0, 0.04f, 255, 15);
  TEST_ASSERT_TRUE(slow > 85);
  for (int i = 0; i < 200; ++i) pi.update(100, 0, 0.04f, 120, 15);
  TEST_ASSERT_EQUAL_INT16(120, pi.update(100, 0, 0.04f, 120, 15));
  TEST_ASSERT_TRUE(pi.integral <= 40.0f);
}

void test_pi_never_reverses_to_slow_down() {
  WheelPi pi = makePi();
  TEST_ASSERT_TRUE(pi.update(50, 400, 0.04f, 255, 15) >= 0);
  TEST_ASSERT_TRUE(pi.update(-50, -400, 0.04f, 255, 15) <= 0);
}

void test_parse_simple_commands() {
  TEST_ASSERT_EQUAL(CMD_PING, parse("PING").type);
  TEST_ASSERT_EQUAL(CMD_ARM, parse("ARM").type);
  TEST_ASSERT_EQUAL(CMD_DISARM, parse("DISARM\r").type);
  TEST_ASSERT_EQUAL(CMD_STOP, parse("STOP").type);
  TEST_ASSERT_EQUAL(CMD_CAL, parse("CAL").type);
  TEST_ASSERT_EQUAL(CMD_NONE, parse("").type);
  TEST_ASSERT_EQUAL(CMD_NONE, parse("   ").type);
}

void test_parse_vel() {
  Command c = parse("VEL 250 -80 800");
  TEST_ASSERT_EQUAL(CMD_VEL, c.type);
  TEST_ASSERT_EQUAL(250, c.a[0]);
  TEST_ASSERT_EQUAL(-80, c.a[1]);
  TEST_ASSERT_EQUAL(800, c.a[2]);
}

void test_parse_rejects_bad_input() {
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("VEL 1 2").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("VEL 1 2 3 4").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("VEL 1 2 x").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("VEL 1.5 2 3").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("VEL 501 0 0").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("VEL 0 0 3001").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("PING 1").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("ping").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("FOO").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("TRAY 91").type);
  TEST_ASSERT_EQUAL(CMD_INVALID, parse("TRAY -1").type);
}

void test_parse_tray_and_oled() {
  Command c = parse("TRAY 50");
  TEST_ASSERT_EQUAL(CMD_TRAY, c.type);
  TEST_ASSERT_EQUAL(50, c.a[0]);
  c = parse("OLED TO ELECTRICAL");
  TEST_ASSERT_EQUAL(CMD_OLED, c.type);
  TEST_ASSERT_EQUAL_STRING("TO ELECTRICAL", c.text);
}

void test_line_buffer() {
  LineBuffer lb;
  const char* text = "PING\n";
  LineBuffer::Result r = LineBuffer::PENDING;
  for (const char* p = text; *p; ++p) r = lb.feed(*p);
  TEST_ASSERT_EQUAL(LineBuffer::LINE, r);
  TEST_ASSERT_EQUAL_STRING("PING", lb.line());

  // An over-long line is dropped whole, and the next line is read normally.
  for (int i = 0; i < 100; ++i) TEST_ASSERT_EQUAL(LineBuffer::PENDING, lb.feed('A'));
  TEST_ASSERT_EQUAL(LineBuffer::OVERFLOW_LINE, lb.feed('\n'));
  for (const char* p = "STOP"; *p; ++p) lb.feed(*p);
  TEST_ASSERT_EQUAL(LineBuffer::LINE, lb.feed('\n'));
  TEST_ASSERT_EQUAL_STRING("STOP", lb.line());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_inverse_forward);
  RUN_TEST(test_inverse_strafe_left);
  RUN_TEST(test_inverse_rotate_ccw);
  RUN_TEST(test_limit_keeps_ratio);
  RUN_TEST(test_step_toward);
  RUN_TEST(test_pi_zero_target_brakes_and_clears);
  RUN_TEST(test_pi_feed_forward_at_speed);
  RUN_TEST(test_pi_pushes_harder_when_slow_and_respects_cap);
  RUN_TEST(test_pi_never_reverses_to_slow_down);
  RUN_TEST(test_parse_simple_commands);
  RUN_TEST(test_parse_vel);
  RUN_TEST(test_parse_rejects_bad_input);
  RUN_TEST(test_parse_tray_and_oled);
  RUN_TEST(test_line_buffer);
  return UNITY_END();
}
