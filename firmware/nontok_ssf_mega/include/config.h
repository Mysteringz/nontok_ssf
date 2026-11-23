// Pin map and constants for the Nontok SSF Mega firmware (PLAN.md sections 3.2 and 6).
// Pins of existing hardware are copied from ../Nontok (docs/hardware.md, core config.h) and must
// stay as wired: the same robot also runs the parking task. Values marked TODO need the robot.
#pragma once
#include <Arduino.h>

// ---------------- Serial ----------------
const unsigned long USB_BAUD = 115200;
const unsigned long BLUETOOTH_BAUD = 9600;  // HC-06 on Serial3 (D14/D15)

// ---------------- Timing ----------------
const uint16_t CONTROL_PERIOD_MS = 40;   // 25 Hz
const uint16_t CMD_WATCHDOG_MS = 300;    // armed with no VEL for this long -> stop and disarm
const uint8_t RNG_EVERY_CYCLES = 5;      // 5 Hz
const uint8_t STA_EVERY_CYCLES = 12;     // ~2 Hz

// ---------------- Motors (MOEBIUS shield, TB6612) ----------------
// Order: front-left/M1, front-right/M2, rear-left/M3, rear-right/M4.
const uint8_t MOTOR_PWM[4] = {12, 8, 9, 5};
const uint8_t MOTOR_IN1[4] = {34, 37, 43, A4};
const uint8_t MOTOR_IN2[4] = {35, 36, 42, A5};
// Measured 23 Oct in ../Nontok with the wheels raised: positive = toward the ultrasonic end.
const int8_t MOTOR_SIGN[4] = {1, -1, 1, -1};
const uint8_t MIN_PWM = 60;   // below this the motors stall
const uint8_t MAX_PWM = 120;  // commissioning cap; raise only after checkpoint CP1

// ---------------- Wheel geometry and control ----------------
const float WHEEL_DIAMETER_MM = 80.0f;       // calipers 23 Nov; refine with the 2 m run (CP2)
const float ENC_COUNTS_PER_REV = 660.0f;     // 11 pulses x 30 gearbox x 2 edges; TODO verify
const float MM_PER_COUNT = 3.14159265f * WHEEL_DIAMETER_MM / ENC_COUNTS_PER_REV;
const float ROTATION_ARM_MM = 192.5f;        // half wheelbase + half track = 165/2 + 220/2 (23 Nov)
const float MAX_WHEEL_MM_S = 400.0f;
const float WHEEL_DEAD_MM_S = 15.0f;         // smaller targets are treated as stop
const float ACCEL_MM_S2 = 300.0f;
const float ACCEL_RAD_S2 = 1.5f;
// Feed-forward PWM = FF_A + FF_B * |speed|, per wheel (FL, FR, RL, RR).
// TODO: fit from FFTEST with the wheels raised (tools/wheel_bench.py ff prints these lines).
const float FF_A[4] = {50.0f, 50.0f, 50.0f, 50.0f};
const float FF_B[4] = {0.35f, 0.35f, 0.35f, 0.35f};
const float WHEEL_KP = 0.15f;   // PWM per mm/s
const float WHEEL_KI = 0.6f;    // PWM per mm/s per second
const float WHEEL_I_MAX = 40.0f;

// ---------------- Encoders ----------------
// Channel A on external interrupts: D18 (INT3), D19 (INT2), D3 (INT5), D2 (INT4).
// Channel B: D31 (PC6), D38 (PD7), D49 (PL0), A1 (PF1).
const uint8_t ENC_A_PIN[4] = {18, 19, 3, 2};
const uint8_t ENC_B_PIN[4] = {31, 38, 49, A1};
// +1 if the count rises when the wheel rolls the robot forward. TODO verify at CP1.
const int8_t ENC_SIGN[4] = {1, -1, 1, -1};
// The Mega's A1 header pin is reported broken, so wheel 4 takes its direction from the sign of
// its last PWM command. Set to 0 once channel B of M4 is confirmed readable.
#ifndef M4_DIR_FROM_COMMAND
#define M4_DIR_FROM_COMMAND 1
#endif

// ---------------- Ultrasonics ----------------
// Index: 0 left 45 deg, 1 right 45 deg, 2 front, 3 left side (+90), 4 right side (-90).
const uint8_t US_COUNT = 5;
const uint8_t US_TRIG[US_COUNT] = {48, 40, 22, 10, 6};
const uint8_t US_ECHO[US_COUNT] = {47, 41, 24, 11, 7};
// Consecutive pings face 90-135 degrees apart (same order as the ../Nontok probe).
const uint8_t US_PING_ORDER[US_COUNT] = {0, 4, 2, 3, 1};
const unsigned long US_TIMEOUT_US = 15000;  // about 2.5 m
const int16_t US_MIN_MM = 20;

// ---------------- Gyro (MPU6050 on the shield IMU header, D20/D21) ----------------
const uint8_t IMU_ADDRESS = 0x68;
const float GYRO_SIGN = -1.0f;  // counter-clockwise positive; confirmed 27 Oct in ../Nontok
const unsigned long I2C_TIMEOUT_US = 25000;

// ---------------- Tray stepper (dump) ----------------
// Stepper on the shield PL header through include/stepper_driver.h (from ../Nontok, wiring
// verified 21 Nov): STEP D46, DIR D45, EN D44, 1/16 microstepping = 3200 steps per turn.
// Open loop: 0 deg is where the tray is at power-up, so power the robot with the tray flat.
// TRAY <deg> is in stepper degrees from there. 0..130 is the travel ../Nontok allows
// (STEPPER_MAX_DEG, 130 = dump). The parser refuses targets outside it (ERR UNKNOWN);
// traySetTarget clips as a second guard. Keep equal to TRAY_LIMIT_DEG in nontok_core.h.
const uint8_t TRAY_MAX_DEG = 130;
const bool TRAY_STEPPER_INVERT = false;      // true if a positive move lowers the tray
const uint16_t TRAY_STEP_PERIOD_US = 400;    // cruise, 0.78 turn/s (../Nontok default)
const uint16_t TRAY_START_PERIOD_US = 1600;  // first and last step of the ramp
const uint16_t TRAY_RAMP_STEPS = 400;
const uint16_t TRAY_RELEASE_MS = 500;        // driver off this long after reaching 0 deg

// ---------------- Voltage sensing (optional) ----------------
// Set to 1 once the divider is wired; otherwise the value is reported as 0.
#ifndef CHARGE_SENSE_FITTED
#define CHARGE_SENSE_FITTED 0
#endif
#ifndef BATTERY_SENSE_FITTED
#define BATTERY_SENSE_FITTED 0
#endif
const uint8_t CHARGE_SENSE_PIN = A8;
const uint8_t BATTERY_SENSE_PIN = A9;
const float CHARGE_DIVIDER = 2.0f;   // receiver volts / pin volts; TODO match the resistors
const float BATTERY_DIVIDER = 2.0f;

// ---------------- Display (optional) ----------------
#ifndef OLED_ENABLED
#define OLED_ENABLED 0
#endif
