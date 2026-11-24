# CP1 results — Mega firmware and wheel control (23–24 Nov 2026)

Firmware on the Mega: `ssf_db40d8a` (branch `firmware-cp1-fixes`). Raw logs on the Jetson in
`~/Documents/David/wheel_bench/`. Tool: `tools/wheel_bench.py` (runs on the Jetson).

## Hardware events

- **23 Nov:** a live power lead touched the metal chassis while wiring; the Mega was destroyed.
  A new Mega 2560 was fitted and everything rewired: see [wiring.md](wiring.md).
- The new Mega's EEPROM is empty: the teammate's field calibration for the competition run is **not**
  on it. A saved copy is `../Nontok/REAL Run Prog/field-2026-11-03b.json` (upload with their
  `tools/upload_calibration.py` while their probe dashboard is connected).
- The teammate's probe firmware was replaced by ours. Restore: `tools/mega_firmware.sh probe` on the
  Jetson; back to ours: `tools/mega_firmware.sh ssf`.

## Wheels raised

| Test | Result |
|---|---|
| Hand test (`watch`) | 660 counts per wheel turn on all four; each wheel moves only its own counter. `ENC_SIGN` negated to {−1, 1, −1, 1} |
| Motor directions (probe `SPIN`) | + turns FL and RL forward, FR and RR backward, as `MOTOR_SIGN` expects |
| `ff --levels 30 40 50 60 90 120` | Every wheel turns from PWM 30 both ways; speed linear in PWM. Fit PWM = FF_A + FF_B·v: FF_A {2.8, 3.9, 5.4, 5.3}, FF_B {0.148, 0.150, 0.146, 0.143}. Forward/reverse differ 5–11 % (left wheels faster in reverse, right wheels faster forward). Top speed at PWM 120: 750–840 mm/s |
| `hold` (100/200/300 mm/s both ways, strafe 150, turn) | **8/8 pass**, every wheel within about 1 % of its target; RR direction confirmed by eye |
| Ctrl-C stop | Interrupt at 200 mm/s: `!` sent and the script exited in 0.05 s; wheels stopped at once (by eye) |
| Tray stepper | 120° up and back, no lost steps (probe firmware); dump range 0–130° |

The feed-forward was fitted with the wheels **raised**: on the floor the load adds offset, so refit
at CP2 (the PI loop's integral covers the difference meanwhile).

## On the floor

| Test | Result |
|---|---|
| `drive --distance 1000` | Encoders +1010 mm (FL 1009, FR 1006, RL 1012, RR 1013); tape **1 m**, so the 80 mm wheel diameter holds. Gyro: +4.6° of rotation over the metre (roller slip, to check at CP2) |
| Obstacle stop | First try faced a wall at 93 cm: the front reading fell 930 → 523 mm, and a stray 61 mm FL45 echo stopped it. The check now needs two close readings in a row |

## Still open for CP1

- USB-unplug stop test (pull the USB during a 100 mm/s `hold`; the Mega must stop within 0.3 s).
- Ultrasonic board test at 300 and 1000 mm (±20 mm).
- 30-minute run.

## Sensors after the rewiring

- Gyro: mounted flat, chips down (accel Z ≈ −14800); bias −47.6 raw, noise 2.65 raw at rest.
- Ultrasonics: all five echo; `-1` means nothing within about 3 m. FL45 gives occasional stray short
  readings at its angle.
