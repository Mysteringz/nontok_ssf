# Mega wiring on the MOEBIUS shield (as rewired 23–24 Nov 2026)

The Mega 2560 sits under the **MOEBIUS TECH 4Motor&9Servo** shield (TB6612 drivers, updated
2024-07 revision: M4 direction on A4/A5). Pin names below are the **labels printed on the shield**
(port names such as PL1), with the Arduino pin in brackets. Sources: the manufacturer's 2024-07
schematic, `../Nontok/docs/hardware.md`, and the checks of 23–24 Nov after the Mega was replaced.

All signals were checked live on 24 Nov: motors and encoders (each wheel moves only its own counter,
directions by eye), five ultrasonics (echo when an object is within ~3 m), gyro (WHO_AM_I 0x68),
OLED (0x3c), tray stepper (120° up and back).

## Safety first

- **Switch the battery off before touching any wire.** On 23 Nov a live power lead touched the metal
  chassis and destroyed the Mega.
- The **Servo header's middle row is battery voltage (6–8.4 V), not 5 V.** No sensor VCC there: plug
  only signal and GND into the Servo header and take VCC from a 5 V pin (DIGITAL or Analog header).
- **PL3, PL4, PL5 belong to the tray stepper** (both firmwares drive them as outputs). A sensor output
  on them fights the Mega's output.
- A sensor with its signal wires connected but **no VCC/GND** can hold the bus or read nothing: an
  unpowered gyro pulls SDA/SCL low and hangs the I²C scan.
- Before power-on: continuity beeper between 5 V / BATT and GND must stay silent.

## Headers on the shield

Each Servo, DIGITAL and Analog channel is a column of three pins: **signal / power / GND**.

### Motor plugs M1–M4 (6 pins each)

| Pin | M1 front-left | M2 front-right | M3 rear-left | M4 rear-right |
|---|---|---|---|---|
| 6 | M1+ | M2+ | M3+ | M4+ |
| 5 | 5 V | 5 V | 5 V | 5 V |
| 4 | encoder A, TXD1 (D18) | encoder A, RXD1 (D19) | encoder A, PE5 (D3) | encoder A, PE4 (D2) |
| 3 | encoder B, PC6 (D31) | encoder B, PD7 (D38) | encoder B, PL0 (D49) | encoder B, ADC1 (A1) — **broken** |
| 2 | GND | GND | GND | GND |
| 1 | M1− | M2− | M3− | M4− |

PWM / direction (fixed on the shield): M1 D12 / D34, D35; M2 D8 / D37, D36; M3 D9 / D43, D42;
M4 D5 / A4, A5. Raw PWM + turns M1 and M3 forward and M2 and M4 backward (`MOTOR_SIGN`).

### DIGITAL header (signal / 5 V / GND)

| Row | Label | Device |
|---|---|---|
| 1 | PL1 (D48) | front-left 45° ultrasonic TRIG |
| 2 | PL2 (D47) | front-left 45° ultrasonic ECHO |
| 3 | PG0 (D41) | front-right 45° ultrasonic **ECHO** |
| 4 | PG1 (D40) | front-right 45° ultrasonic **TRIG** |
| 5 | PC4 (D33) | gyro INT |
| 6 | PC5 (D32) | colour OUT (not fitted now) |
| 7 | PC7 (D30) | colour S2 (not fitted now) |
| 8 | PA7 (D29) | colour S3 (not fitted now) |
| 9 | PA6 (D28) | colour S0 (not fitted now) |
| 10 | PA3 (D25) | colour S1 (not fitted now) |

Rows 3/4 look swapped but are correct (TRIG D40 = PG1). The schematic prints "40" next to PG0; go by
the port label. The 5 V pins of free rows power the side ultrasonics.

### Servo header (signal / **6–8.4 V** / GND)

| Row | Label | Device |
|---|---|---|
| 1 | PG5 (D4) | free |
| 2 | PH3 (D6) | right-side 90° ultrasonic TRIG |
| 3 | PH4 (D7) | right-side 90° ultrasonic ECHO |
| 4 | PB4 (D10) | left-side 90° ultrasonic TRIG |
| 5 | PB5 (D11) | left-side 90° ultrasonic ECHO |
| 6 | PB7 (D13) | free (on-board LED) |
| 7 | PL3 (D46) | tray stepper STEP |
| 8 | PL4 (D45) | tray stepper DIR |
| 9 | PL5 (D44) | tray stepper EN (LOW = on; HIGH at boot) |

### Long 2×18 DIGITAL header (mirror of the Mega's D22–D53)

| Pin | Device |
|---|---|
| PA0 (22) | front ultrasonic TRIG |
| PA2 (24) | front ultrasonic ECHO |
| 5 V / GND (ends) | front ultrasonic VCC / GND |

Do not use 31, 38, 49 here (encoder B) or the 4-pin "Ultrasonic" header (its pins are D24/D25;
D25 is colour S1).

### Small headers

| Header | Pins | Device |
|---|---|---|
| IMU (5 pins) | SDA (D20), SCL (D21), GND, VCC 5 V, one not connected | MPU6050 (GY-521). Match names, not positions. XDA/XCL/AD0 open (address 0x68) |
| IIC (4 pins) | SDA, SCL, VCC, GND | OLED 128×32 SSD1306, 0x3c (shares the bus with the gyro) |
| COM3 (4 pins) | TXD3 (D14), RXD3 (D15), GND, 5 V | Bluetooth HC-06 "DIVA", 9600 baud: module RXD → TXD3, module TXD → RXD3 |
| Analog | A0, A2, A7, A6 | LDRs left → right (not needed by nontok_ssf). A1/A4/A5: leave empty (M4) |
| Analog | A15 | teammate's start button, to GND (never to 5 V) |
| USB-B (Mega) | — | Jetson, `/dev/nontok_mega` |

## Devices, pin by pin

### Ultrasonics (HC-SR04: VCC, Trig, Echo, GND)

| Sensor | Trig | Echo | VCC | GND |
|---|---|---|---|---|
| Front-left +45° | PL1 | PL2 | 5 V of DIGITAL row 1–2 | GND of DIGITAL row 1–2 |
| Front-right −45° | PG1 | PG0 | 5 V of DIGITAL row 3–4 | GND of DIGITAL row 3–4 |
| Front 0° | PA0 | PA2 | 5 V of the 2×18 header | GND of the 2×18 header |
| Left side +90° | PB4 | PB5 | 5 V of a free DIGITAL row (e.g. PC5) | GND of Servo row 4–5 |
| Right side −90° | PH3 | PH4 | 5 V of a free DIGITAL row (e.g. PC7) | GND of Servo row 2–3 |

Firmware order (`RNG` line and `US_TRIG`/`US_ECHO` in config.h): FL45, FR45, front, left side,
right side. `-1` = no echo, i.e. nothing within about 3 m.

### Gyro (MPU6050 breakout)

| Breakout | Shield |
|---|---|
| VCC | IMU VCC (5 V) |
| GND | IMU GND |
| SCL | IMU SCL |
| SDA | IMU SDA |
| INT | PC4 (DIGITAL row 5) |

**Mounting:** flat, chips facing **down** (accelerometer Z about −15000 at rest), edges square with
the robot. `GYRO_SIGN = −1` was calibrated for that orientation; chip-up reverses every turn, and on
its edge (gravity on X) the gyro barely sees the robot turn. Keep the robot still for a few seconds
after every Mega reset so the bias is taken.

### Tray stepper (DRV8825/A4988 on the "42" driver board)

| Driver board | Shield |
|---|---|
| STEP | PL3 |
| DIR | PL4 |
| EN | PL5 |
| GND | GND next to PL3 |
| motor power | 12 V, in parallel with the shield's battery input (mind polarity) |
| motor | 4 wires to 1A 1B 2A 2B (coil pairs together) |

DIP switches at 1/16 microstepping (3200 steps per turn). Range 0–130° from the power-up position, so
power up with the tray flat. Wrong direction: `TRAY_STEPPER_INVERT` in config.h, not a rewire.

### OLED (0.91" SSD1306 128×32): GND, VCC (5 V), SCL, SDA on the IIC header, matched by name. Found
only at boot: plug it in with the power off.
