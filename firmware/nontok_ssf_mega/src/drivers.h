// Hardware drivers of the Nontok SSF firmware. One .cpp per block.
#pragma once
#include <Arduino.h>

// ---- encoders.cpp ----
void encodersBegin();
void encodersRead(int32_t counts[4]);        // atomic copy; positive = wheel rolls forward
void encodersSetCommandDir(uint8_t wheel, int8_t dir);  // used for wheels without channel B

// ---- motors.cpp ----
void motorsBegin();
void motorsWrite(const int16_t pwm[4]);      // signed, positive = wheel forward; 0 = brake
void motorsBrake();
void motorsCoast();                          // outputs off: the robot can be pushed by hand

// ---- ultrasonic.cpp ----
void ultrasonicBegin();
void ultrasonicPingNext();                   // one sensor per call, blocks up to ~15 ms
const int16_t* ultrasonicMm();               // US_COUNT values, -1 = no valid echo

// ---- imu.cpp ----
void imuBegin();
// Call every control cycle. `still` = the robot is known to be stationary.
void imuService(unsigned long nowMs, bool still);
int16_t imuTakeRateMrad();                   // mean yaw rate since the last call, CCW positive
bool imuOk();                                // device answers and a bias has been measured
void imuRequestBias();                       // re-measure the bias the next time the robot is still
uint16_t imuTimeouts();

// ---- tray.cpp ----
void trayBegin();                            // stepper driver off; the current position is 0 deg
void traySetTarget(int16_t deg);             // clipped to 0..TRAY_MAX_DEG
void trayStop();                             // emergency stop: no DONE TRAY, the position is kept
bool trayService(unsigned long nowMs);       // call every loop() pass; true once when the target is reached
bool trayMoving();
uint8_t trayDeg();

// ---- power.cpp ----
uint16_t chargeMillivolts();
uint16_t batteryMillivolts();

// ---- display.cpp ----
void displayBegin();
void displayText(const char* text);
