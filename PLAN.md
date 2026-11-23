# Nontok SSF — Self-Return Robot: Plan, Spec Sheet and Checkpoints

Version 1.2, 12 November 2026. Section 14 added: what has been implemented and where it differs from sections 1–13.
Hardware facts are taken from the sibling project `../Nontok` (same robot, without the cameras and tray).
Every value marked **TODO** must be measured on the real robot; every value marked **(assumed)** is a design assumption to confirm.

---

## 1. Summary

Nontok waits on its charging dock, wakes on the spoken word "Nontok", drives to a preset pickup point, waits for an item in its tray, classifies it with the tray camera, drives to the matching return station (electrical, mechanical, scraps or tools), aligns with the box, tips the tray, and returns to the dock.

The design uses only the sensors already on the robot plus four cameras:

- **Where am I?** Printed AprilTags at known positions give absolute position fixes. Wheel encoders and the gyro carry the pose between tag sightings.
- **How do I get there?** Nav2 plans on a hand-drawn map of the lab.
- **What is in the way?** The five ultrasonics feed the Nav2 costmap.
- **Final alignment:** The robot servos directly on the tag at each station and at the dock.

No LiDAR, no SLAM, no new sensors. Budget cap: HK$500.

### Constraints that shape the design

| Constraint | Consequence |
|---|---|
| No LiDAR, HK$500 cap | Tag-based localisation; the map is drawn by hand |
| Ultrasonics cover only the front 180° | Robot drives nose-first in transit; reversing only for short, slow, fixed moves |
| Same robot serves the `../Nontok` parking task | **No rewiring of existing sensors.** New firmware uses the existing pins; new devices go on free pins only |
| One Mega runs one sketch at a time | A separate sketch `nontok_ssf_mega` is flashed for this task |
| Mecanum wheels slip, especially sideways | Heading comes from the gyro, not the wheels; strafing is limited to docking |

### Assumptions to confirm first

| # | Assumption | Why it matters |
|---|---|---|
| A1 | The computer is a **Jetson Orin Nano** (JetPack 6, Ubuntu 22.04). `../Nontok/README.md` says "Jetson Nano" with a JetPack 4 container. | An original Jetson Nano cannot run ROS 2 Humble natively and has far less compute. If it is the old Nano, ROS 2 must run in Docker and the camera rates in §5.3 must be halved. |
| A1 — settled 16 November 2026 | **It is the original Jetson Nano Developer Kit, 4 GB** (JetPack 4.6, L4T R32.6.1, Ubuntu 18.04). | ROS 2 Humble runs in Docker; see section 14.4. Wherever sections 1–13 say Orin Nano, JetPack 6 or a native install, section 14.4 applies instead. |
| A2 | The four cameras are USB UVC webcams that can output 1280×720 MJPEG. | Sets tag range (§5.2) and USB layout (§3.4). |
| A3 | The tray tips over the **ultrasonic end** of the robot, and that end is the front (`base_link` +x). Decided 17 Nov 2026: the top assembly is turned round so that the tray's tipping edge and the ultrasonics are at the same end. | The robot drives, senses and tips toward the same end, so stations are approached nose-first with the front cameras and the front ultrasonic. The two tag cameras must also be at that end, one at the other. `back_in: true` in `stations.yaml` remains available for a station that must be reversed into (rear camera, nothing senses behind). |
| A4 | One preset pickup point. | The wake word alone cannot say which bench is calling. Several pickup points is a stretch goal (§11). |
| A5 | One item per trip. | Keeps the classifier and tray logic simple. |
| A6 | The wireless charging dock and receiver from the `../Nontok` proposal (module 3) exist or are being built by that team. | This plan covers docking onto it, not building the charger. |

---

## 2. System architecture

```text
                    ┌───────────────────────── Jetson Orin Nano (ROS 2 Humble) ─────────────────────────┐
 USB mic ──────────►│ nontok_voice        wake word "Nontok"                                             │
 Tray camera ──────►│ nontok_perception   tray monitor + item classifier                                 │
 Front-L camera ───►│ nontok_localization tag detectors ─► tag localiser ─► EKF (map→odom)               │
 Front-R camera ───►│                     wheel odom + gyro ─► EKF (odom→base_link)                      │
 Rear camera ──────►│ nontok_navigation   Nav2: planner, MPPI controller, costmap (static + range)       │
                    │ nontok_docking      tag servo for stations and dock                                │
                    │ nontok_mission      state machine (the seven task steps)                           │
                    │ nontok_base         serial bridge  ◄──────── USB serial 115200 ────────┐           │
                    └────────────────────────────────────────────────────────────────────────│───────────┘
                                                                                             ▼
                    ┌───────────────────────── Arduino Mega 2560 (nontok_ssf_mega) ─────────────────────┐
                    │ 4× wheel velocity PID (encoders)   5× ultrasonic   MPU6050 gyro   tray servo      │
                    │ command watchdog   arming   charge-voltage sense                                  │
                    └───────────────────────────────────────────────────────────────────────────────────┘
```

Division of work: the Mega does everything time-critical (wheel control, sensor timing, safety stop). The Jetson does everything that needs a camera, a map or a decision.

---

## 3. Hardware spec sheet

### 3.1 Existing hardware (from `../Nontok`)

| Item | Detail | Source / status |
|---|---|---|
| Controller | Arduino Mega 2560 (clone, CH340 USB, VID:PID 1A86:7523) | Board replaced 27 Oct |
| Motor shield | MOEBIUS TECH 4Motor&9Servo, 2× TB6612FNG, updated (2024) revision | `docs/hardware.md` |
| Wheels | 4× mecanum, 80 mm diameter (kit value) | **TODO** measure diameter |
| Motors | DC gear motors with 11-pulse encoders, 1:30 gearbox (kit value) | **TODO** verify by turning a wheel 10 revolutions |
| Rotation arm | 160 mm (half wheelbase + half track) | Placeholder in `config.h`; **TODO** measure |
| Gyro | MPU6050 at I²C 0x68, on the shield IMU header | Confirmed 27 Oct |
| Ultrasonics | 5× HC-SR04-type at +90°, +45°, 0°, −45°, −90° | Confirmed 27 Oct |
| OLED | SSD1306 128×32, I²C 0x3c, no reset wire | Confirmed |
| Bluetooth | HC-06-style on Serial3, 9600 baud, name DIVA | Confirmed; useful as a wireless stop |

Measured behaviour worth designing around:

- The robot rolled about 20 cm after a stop from 380 mm/s (2 Nov). Transit speed is capped well below that.
- The gyro reads about 0.9 % short (scale 0.991) and drifts about 0.003 °/s when still.
- Motor noise on the I²C bus hung the Mega until a 25 ms Wire timeout and bus recovery were added. The new sketch must keep both.
- Opening the USB serial port resets the Mega.

### 3.2 Mega pin map for `nontok_ssf_mega`

Existing pins, used as they are wired today:

| Function | Pins | Notes |
|---|---|---|
| M1 front-left: PWM / IN1 / IN2 | D12 / D34 / D35 | `MOTOR_SIGN = {+1, −1, +1, −1}` (measured 23 Oct, forward = toward the ultrasonic end) |
| M2 front-right: PWM / IN1 / IN2 | D8 / D37 / D36 | |
| M3 rear-left: PWM / IN1 / IN2 | D9 / D43 / D42 | |
| M4 rear-right: PWM / IN1 / IN2 | D5 / A4 / A5 | |
| Encoder channel A, M1–M4 | D18 / D19 / D3 / D2 | External interrupts; already counted by the probe firmware |
| Encoder channel B, M1–M4 | D31 / D38 / D49 / A1 | **Not read by any current firmware.** A1 header pin is reported broken, see §6.2 |
| Ultrasonic left 45°: TRIG / ECHO | D48 / D47 | |
| Ultrasonic right 45°: TRIG / ECHO | D40 / D41 | |
| Ultrasonic front: TRIG / ECHO | D22 / D24 | |
| Ultrasonic left side (+90°): TRIG / ECHO | D10 / D11 | Signal on SERVO row; VCC from 5 V |
| Ultrasonic right side (−90°): TRIG / ECHO | D6 / D7 | Signal on SERVO row; VCC from 5 V |
| MPU6050: SDA / SCL / INT | D20 / D21 / D33 | `GYRO_SIGN = −1` |
| OLED | D20 / D21 | Optional status display |
| Bluetooth | D14 / D15 (Serial3) | Optional remote STOP |
| Jetson link | D0 / D1 (USB Serial0) | 115200 baud |

Pins that belong to the other task and stay untouched (inputs, not driven by this sketch):

| Function | Pins |
|---|---|
| Four LDRs | A0, A2, A7, A6 |
| Colour sensor S1 / S0 / S3 / S2 / OUT | D25 / D28 / D29 / D30 / D32 |

New pins proposed for this task:

| Function | Pin | Notes |
|---|---|---|
| Tray dump servo signal | **D46** | Confirm D46 is exposed on the shield SERVO row; alternatives D44, D45. Uses Timer5, which no motor PWM pin needs |
| Tray-home switch (optional) | D26 | Free on the updated shield (legacy M4 pin) |
| Charge-receiver voltage sense | **A8** | Through a resistor divider; A0 is taken by an LDR |
| Motor battery voltage sense (optional) | A9 | Through a divider; for a low-battery return |

### 3.3 New hardware to add

| Item | Spec | Mounting |
|---|---|---|
| Front cameras ×2 | Identical, 1280×720 MJPEG, parallel optical axes, **baseline 120 mm (assumed)**, rigid 3D-printed holder | Front, lens height about 150–250 mm, level |
| Rear camera ×1 | Same resolution | Rear centre, level |
| Tray camera ×1 | 640×480 is enough | Above the tray looking down, whole tray floor in view |
| Tray | Matte, single-colour floor (pale grey or blue recommended), hinged at the front edge, lip about 30 mm | Top deck |
| Tray servo | MG996R-class metal-gear servo, 9–11 kg·cm | Lifts the rear of the tray to about 45–55° |
| Servo supply | 5–6 V, 3 A UBEC from the motor battery | **Do not power the servo from the SERVO row red pin:** it carries raw battery voltage (6–8.4 V), above a typical servo rating |
| Microphone | USB omnidirectional | Top deck, away from the motors |
| Tags | AprilTag 36h11, 160 mm black square on A4 with a white border, matte paper on foam board | See §5.2 |

### 3.4 Jetson connections

| Device | Connection | Notes |
|---|---|---|
| Mega | USB-A port directly on the Jetson | Appears as `/dev/ttyUSB0` (CH340), not `ttyACM0`. Add a udev rule to create `/dev/nontok_mega` |
| Front-L, Front-R cameras | Separate USB-A ports on the Jetson | Use MJPEG; two raw YUYV cameras on one USB 2 bus often fail to start |
| Rear camera, tray camera, microphone | Powered USB hub | Six devices exceed the four USB-A ports |
| Camera naming | udev symlinks by physical port (`/dev/v4l/by-path/...`) | Identical cameras usually share a serial number, so name by port |

### 3.5 Power

| Rail | Supplies | Notes |
|---|---|---|
| Jetson pack | Jetson, cameras, hub, microphone | Unchanged |
| Motor Li-Po (2S, 6–8.4 V per the shield label) | Motors, UBEC → tray servo | Fit a physical cut-off switch on this line as the emergency stop |
| Mega 5 V | Ultrasonics, gyro, OLED | Unchanged |
| Common ground | Mega, UBEC, servo | Required for the servo signal |

### 3.6 Budget (cap HK$500)

Prices are rough estimates to check before ordering.

| Item | Needed if | Estimate (HK$) |
|---|---|---|
| MG996R-class servo | Always | 30–45 |
| 5–6 V 3 A UBEC | Always | 20–30 |
| Foam board, A4 matte prints, tape for about 16 tags | Always | 40–60 |
| Tray material and hinge | Not 3D-printed from existing stock | 20–40 |
| Powered USB hub | No hub available | 60–100 |
| USB microphone | None available | 30–60 |
| LED strip for the tray camera | Tray lighting proves uneven (CP7) | 15–25 |
| Micro switch, resistors, wire | Optional | 10–20 |
| **Total** | | **about 225–380**, leaving at least HK$120 in reserve |

---

## 4. Software stack

| Layer | Choice | Reason |
|---|---|---|
| OS | JetPack 6.x (Ubuntu 22.04), headless | Matches ROS 2 Humble binaries |
| Middleware | ROS 2 Humble | Nav2 and `robot_localization` install from apt |
| Mega firmware | PlatformIO, Arduino framework, `megaatmega2560` | Same toolchain as `../Nontok` |
| Tag detection | AprilTag 36h11 via the `apriltag` C library, one small node per camera reading the camera directly with OpenCV | Publishing only detections avoids sending 720p images through ROS, which is heavy on this board |
| State estimation | `robot_localization`, two EKFs | Standard odom / map split |
| Navigation | Nav2: Smac 2D planner, MPPI controller, static + range + inflation costmap layers | Transit is forward-only, so MPPI runs its DiffDrive model; sideways motion is used only by the docking node. DWB is the fallback controller if MPPI is too slow |
| Wake word | openWakeWord with a custom "Nontok" model | Free, runs on CPU |
| Classifier | Ultralytics YOLO11n-cls trained on a PC or Colab, exported to ONNX, run with ONNX Runtime on CPU | A 224 px nano classifier does not need the GPU, which removes the TensorRT setup risk. TensorRT is an optional later step |
| Mission logic | Python state machine using Nav2 Simple Commander | Easier to debug than a behaviour tree for seven steps |
| Remote view | `foxglove_bridge` on the Jetson, Foxglove on the Mac | ROS 2 does not run natively on macOS; develop over SSH |

### Workspace layout (to be created in this folder)

```text
Nontok_ssf/
  PLAN.md                      this file
  firmware/nontok_ssf_mega/    PlatformIO project for the Mega
  ros2_ws/src/
    nontok_interfaces/         custom messages, services and the docking action
    nontok_base/               serial bridge: /cmd_vel in; odometry, IMU, ranges, tray service out
    nontok_description/        URDF and static transforms (cameras, ultrasonics)
    nontok_localization/       tag detector, tag localiser, EKF configs, tags.yaml
    nontok_navigation/         Nav2 parameters, map image, stations.yaml
    nontok_docking/            tag-servo action server
    nontok_perception/         tray monitor, classifier
    nontok_voice/              wake word node
    nontok_mission/            state machine
    nontok_bringup/            launch files
  training/                    dataset capture, training and export scripts
  tools/                       tag PDF generator, camera calibration, odometry calibration
  docs/                        measured values, calibration records, test logs
```

Starting points in `../Nontok` (copy, do not import, and review before reuse):

| Need | Existing code |
|---|---|
| Encoder interrupts, ultrasonic ping order, I²C timeout and gyro health check | `firmware/hardware_probe_mega/src/main.cpp` |
| Motor mixing, VEL command, 500 ms watchdog, telemetry line | `firmware/lab_assistant_mega/` and `docs/lab_assistant_protocol.md` |
| Tray occupancy check | `jetson/lab_assistant/perception/tray.py` |
| Dataset capture and YOLO training scripts | `jetson/tools/capture_dataset.py`, `training/` |
| Marker docking logic | `jetson/lab_assistant/behaviors/docking.py` |

---

## 5. Localisation and navigation spec

### 5.1 Frames and units

- ROS side: SI units, REP-103 axes (x forward, y left, yaw counter-clockwise), REP-105 frames `map → odom → base_link`.
- `base_link`: centre of the four wheels, on the floor.
- `map` origin: the robot's pose when docked, x pointing the way it faces on the dock.
- Serial link: integer mm/s and mrad/s, converted in `nontok_base`.

### 5.2 Tags

| Parameter | Value | Reasoning |
|---|---|---|
| Family | AprilTag 36h11 | Very low false-positive rate |
| Size | 160 mm black square | Largest that fits A4 with a 25 mm white border |
| Camera mode | 1280×720, MJPEG, exposure fixed and short | Short exposure limits motion blur |
| Usable range | **2.5 m** for position, 1.5 m for heading | With a 60° lens (assumed) the tag is about 70 px wide at 2.5 m; pose noise grows quickly beyond that |
| Maximum viewing angle | 60° off the tag normal | Detection and pose degrade when oblique |
| Height | Tag centre at camera lens height ±100 mm | Keeps the tag near the image centre row |
| Station and dock tags | Two tags side by side, 300 mm apart centre to centre | A pair fixes heading far better than one tag and removes the single-tag flip ambiguity |
| Route tags | One tag at least every 3 m along each route, and one visible after every turn | Bounds dead-reckoning drift between fixes |
| Expected count | About 14–18 tags | 5 stations × 2, plus route tags |
| ID plan | 0–1 dock, 2–3 pickup, 10–11 electrical, 12–13 mechanical, 14–15 scraps, 16–17 tools, 30+ route | Easy to read in logs |

Tag poses go in `tags.yaml` as `id: [x, y, z, yaw]` in the map frame, measured by tape from the dock origin. Target accuracy: ±20 mm, ±2°. A tag that moves must be re-measured.

### 5.3 Tag detection

| Camera | Rate target | Active |
|---|---|---|
| Front-L | ≥ 10 Hz | Always while moving |
| Front-R | ≥ 10 Hz | Always while moving; drop to 5 Hz if CPU is short |
| Rear | ≥ 5 Hz | Always while moving |
| Tray | 5 Hz | Only while stationary at pickup and after dumping |

Detector settings to start from: `quad_decimate = 2`, `refine_edges = on`, 2 threads per camera.
Each camera needs an intrinsic calibration (checkerboard, reprojection error < 0.5 px) and a measured mounting transform to `base_link` (±5 mm, ±1°).

The two front cameras are used as two independent tag detectors, which doubles the chance of a clean detection. Stereo depth from the pair is a stretch goal only, because unsynchronised USB cameras give unreliable depth while the robot moves.

### 5.4 State estimation

| Stage | Inputs | Output | Rate |
|---|---|---|---|
| Wheel odometry (`nontok_base`) | Four encoder counts | vx, vy from mecanum kinematics | 25 Hz |
| Local EKF | Wheel vx, vy; gyro yaw rate | `odom → base_link` | 25 Hz |
| Tag localiser | Tag detections, `tags.yaml`, camera transforms | Robot pose in `map` with covariance | Per detection |
| Global EKF | Local odometry; tag poses | `map → odom` | 25 Hz |

Rules inside the tag localiser:

- Reject detections beyond 2.5 m, beyond 60° viewing angle, or with decision margin below a threshold (start at 50).
- Reject a fix that disagrees with the current estimate by more than 0.5 m or 20°, unless three consecutive fixes agree with each other (recovery from being lost).
- Position covariance grows with distance squared (start: σ = 20 mm at 1 m).
- Use the heading from a detection only within 1.5 m, or when two tags are seen in one frame; otherwise fuse position only.
- At startup on the dock, the pose is initialised to the map origin.

Mecanum kinematics (wheel linear speeds, order FL, FR, RL, RR; L = rotation arm):

```text
Inverse:  FL = vx − vy − L·wz      FR = vx + vy + L·wz
          RL = vx + vy − L·wz      RR = vx − vy + L·wz
Forward:  vx = ( FL + FR + RL + RR) / 4
          vy = (−FL + FR + RL − RR) / 4
          wz = (−FL + FR − RL + RR) / (4·L)     (not fused; the gyro provides yaw rate)
```

### 5.5 Map and stations

- `map.pgm` at 0.05 m per cell, drawn from tape measurements of the lab: walls, benches, cabinets and fixed furniture as occupied.
- `stations.yaml` holds, for each of dock, pickup, electrical, mechanical, scraps, tools: an **approach pose** 0.6 m in front of the tag pair facing it, the tag IDs, and the final stand-off distance.
- Mark keep-out areas (under benches, chair zones) as occupied, because the ultrasonics miss thin legs.

### 5.6 Nav2 parameters (starting values)

| Parameter | Value | Reason |
|---|---|---|
| Max forward speed | 0.25 m/s | Measured coasting; tag blur |
| Max reverse speed | 0.0 in transit | No rear obstacle sensing |
| Max strafe speed | 0.0 in transit, 0.08 m/s in docking | Side slip; side sensing is one beam each |
| Max yaw rate | 0.8 rad/s | Tag blur |
| Acceleration limits | 0.3 m/s², 1.5 rad/s² | Wheel slip |
| Controller rate | 20 Hz | |
| MPPI | DiffDrive model (no strafe, no reverse in transit), batch 1000, 40 steps of 0.05 s | Reduce batch first if the loop cannot hold 20 Hz |
| Goal tolerance | 0.10 m, 0.15 rad | Docking does the precise part |
| Footprint | **TODO** measure, plus 30 mm padding | |
| Inflation radius | 0.25 m | |
| Local costmap | 3 m × 3 m rolling, 0.05 m, static + range + inflation | |
| Range layer | Five `sensor_msgs/Range` topics, field of view 0.26 rad (15°), max 1.5 m, mark and clear | Readings beyond 1.5 m are too wide and noisy to mark |
| Recovery | Wait 5 s, then clear costmap, then re-plan; no spin, no back-up | Spin and back-up are unsafe without rear sensing |

### 5.7 Precision docking (stations and charger)

Sequence: Nav2 brings the robot to the approach pose, then `nontok_docking` takes over.

| Parameter | Value |
|---|---|
| Input | Pose of the station tag pair in the front camera, about 10 Hz; front ultrasonic as a distance cross-check |
| Control | Proportional control on lateral offset, distance and heading, all three axes at once |
| Speed limit | 0.08 m/s, 0.3 rad/s |
| Station tolerance | ±15 mm lateral, ±20 mm distance, ±3° |
| Dock tolerance | ±10 mm lateral, ±10 mm distance, ±2° (**TODO** confirm against the charging coil's real tolerance) |
| Dock confirmation | Charge-sense voltage above threshold for 1 s; if not, back off 150 mm and retry, at most 3 times |
| Tag lost | Stop within 0.3 s; after 2 s return to the approach pose and retry once |
| Close-range blind zone | Below about 0.25 m the tag pair may leave the image; finish the last part on encoders and the front ultrasonic |

Stand-off distance at each station is set so the tray lip overhangs the box opening: **TODO** from the tray geometry.

---

## 6. Mega firmware spec (`nontok_ssf_mega`)

### 6.1 Loop

| Item | Value |
|---|---|
| Control period | 40 ms (25 Hz) |
| Each cycle | Read encoder counts → wheel velocity PID → write PWM → one ultrasonic ping → drain gyro FIFO → handle serial |
| Ultrasonic schedule | One sensor per cycle in the order left-45, right-side, front, left-side, right-45 (the probe's order: consecutive pings face 90–135° apart). Each sensor refreshes every 200 ms |
| Echo timeout | 15 ms (about 2.5 m); no echo is reported as −1, never as a far reading |
| Gyro | 200 Hz into the chip FIFO, ±500 °/s, every sample integrated; 25 ms I²C timeout with bus recovery; bias taken only when still |
| Watchdog | Motors stop if no `VEL` arrives for 300 ms; hardware watchdog reset at 1 s |
| Arming | Motors stay off until `ARM`; any fault or `STOP` disarms |
| PWM limits | MIN 60, MAX 120 at first (the commissioning cap in `../Nontok`); raise only after CP1 |

### 6.2 Encoders

| Item | Value |
|---|---|
| Channel A | Interrupt on CHANGE on D18, D19, D3, D2 |
| Direction | Read channel B inside the interrupt for M1–M3 (D31, D38, D49) |
| M4 direction | Channel B is on A1, whose header pin is reported broken. Test it at CP1. If dead, take M4's direction from the sign of its PWM command |
| Resolution | 11 pulses × 30 × 2 edges = **660 counts per wheel revolution** (kit value, **TODO** verify) |
| Distance per count | π × 80 mm / 660 = **0.381 mm** |
| Count rate | About 1,050 counts/s per wheel at 0.4 m/s; no load problem for the Mega |
| Velocity estimate | Counts over a 2-cycle (80 ms) sliding window, because one cycle at 0.1 m/s is only about 10 counts |
| Wheel control | PI plus feed-forward (PWM ≈ k·speed + offset), per wheel |

### 6.3 Serial protocol (115200 baud, ASCII lines, `\n` terminated)

Jetson → Mega:

| Command | Reply | Meaning |
|---|---|---|
| `PING` | `PONG` | Link check |
| `ARM` / `DISARM` | `OK ARM` / `OK DISARM` | Enable or disable motor output |
| `VEL <vx> <vy> <wz>` | none | Body velocity in mm/s, mm/s, mrad/s. Resend at least every 300 ms while moving |
| `STOP` or `!` | `OK STOP` | Stop and disarm immediately |
| `TRAY <deg>` | `OK TRAY`, then `DONE TRAY` | Move the tray servo to an angle at a limited rate |
| `CAL` | `OK CAL` | Re-measure gyro bias (robot must be still, about 1 s) |
| `OLED <text>` | none | Optional: show up to 16 characters on the OLED (state or fault reason) |

Mega → Jetson:

| Line | Rate | Fields |
|---|---|---|
| `READY NONTOK-SSF 1` | After reset | Firmware identity |
| `ODO <t_ms> <c1> <c2> <c3> <c4> <gz>` | 25 Hz | Signed cumulative encoder counts FL, FR, RL, RR; gyro yaw rate in mrad/s |
| `RNG <t_ms> <l45> <r45> <f> <ls> <rs>` | 5 Hz | Ranges in mm, −1 = no valid echo |
| `STA <t_ms> <armed> <tray_deg> <charge_mv> <batt_mv> <fault>` | 2 Hz | Status |
| `ERR <reason>` | On event | `WATCHDOG`, `IMU`, `OVERFLOW`, `UNKNOWN` |
| `EVT <name>` | On event | `WAKE` when the Bluetooth module receives `W` (demo fallback trigger); `BTSTOP` when it receives `Z` (the Mega has already stopped) |

Load: about 3,000 bytes/s against a link capacity of 11,520 bytes/s.
The bridge discards any line with the wrong field count or out-of-range values.

---

## 7. Mission spec

### 7.1 State machine

| State | Does | Exit | On failure |
|---|---|---|---|
| S0 DOCKED | Wake-word node listening; motors disarmed; pose reset to map origin | "Nontok" heard → S1 | — |
| S1 UNDOCK | Arm; reverse 0.30 m at 0.08 m/s; rotate toward the route | Done → S2 | Wheel stall → FAULT |
| S2 TO_PICKUP | Nav2 to pickup approach pose, then dock to the pickup tags | Arrived → S3 | 2 failed attempts → S8 |
| S3 WAIT_ITEM | Tray camera compares against the empty-tray reference | Item present and still for 2 s → S4 | Nothing after 120 s → S8 |
| S4 CLASSIFY | Classify 5 frames, majority vote | Confident → S5 | Low confidence: prompt to reposition, retry twice, then S8 with the item still aboard |
| S5 TO_STATION | Nav2 to the station's approach pose | Arrived → S6 | 2 failed attempts → FAULT |
| S6 ALIGN | Tag servo to the station stand-off | In tolerance for 0.5 s → S7 | 2 failed attempts → FAULT |
| S7 DUMP | Tray up, hold 1.5 s, tray down; tray camera checks empty | Empty → S8 | Not empty: repeat once, then S8 and report |
| S8 TO_DOCK | Nav2 to the dock approach pose | Arrived → S9 | 2 failed attempts → FAULT |
| S9 DOCK | Tag servo onto the coil; confirm charge voltage | Charging → S0 | 3 failed attempts → FAULT |
| FAULT | Stop, disarm, show the reason on the OLED and log it | Operator places the robot on the dock → S0 | — |

A `STOP` from the keyboard, Bluetooth or the cut-off switch works in every state.

### 7.2 Wake word

| Item | Spec |
|---|---|
| Engine | openWakeWord, custom model for "Nontok", 16 kHz mono |
| Active | Only in S0, so motor noise is not a problem |
| Target | ≥ 90 % detection at 3 m at normal speaking volume; ≤ 1 false trigger per hour of lab noise |
| Training data | Synthetic speech from the openWakeWord training notebook, plus about 50 real recordings from team members for testing |
| Risk | "Nontok" is not an English word, so synthetic voices may pronounce it differently from the team |
| Fallbacks | (1) Picovoice Porcupine custom keyword; (2) a longer phrase such as "Hey Nontok"; (3) a keyboard or Bluetooth trigger, kept permanently for demos |

### 7.3 Item detection in the tray

| Item | Spec |
|---|---|
| Reference | Image of the empty tray, captured at the pickup point at the start of S3 |
| Present | Mean absolute difference inside the tray region above a threshold (**TODO** tune) |
| Stable | Frame-to-frame difference below a threshold for 2 s (hand has left) |
| Empty check after dump | Same comparison against the reference |

### 7.4 Classifier

| Item | Spec |
|---|---|
| Model | YOLO11n-cls, 224×224 input |
| Classes | One per **item type** (for example resistor, wire offcut, screwdriver, bolt), each mapped to a station in `items.yaml`; plus `empty` |
| Item list | **TODO: team to fix the demo item list, 3–5 item types per station** |
| Dataset | ≥ 150 tray-camera images per item type, varied position, rotation and lighting; 80/20 split by capture session |
| Acceptance | ≥ 95 % station-level accuracy on the held-out session |
| Decision | 5 frames, majority vote, mean confidence ≥ 0.80; otherwise "unknown" |
| Runtime | ONNX Runtime on CPU, target < 100 ms per frame |

Stations are categories, not objects, so the model learns specific item types and a lookup table assigns the station. An item type that was never trained will be misclassified or rejected, which is why the item list must be fixed early.

### 7.5 Dump

| Item | Spec |
|---|---|
| Tray angle | 0° rest, 45–55° tipped (**TODO** tune with real items) |
| Servo rate | About 60 °/s up, 90 °/s down |
| Hold | 1.5 s at the top |
| Robot during dump | Stationary and disarmed |
| Verification | Tray camera empty check; one retry |

---

## 8. Checkpoints

Each checkpoint has a pass test. Do not start a checkpoint until the ones it depends on have passed. Record measured values and results in `docs/`.

Tracks B, C and D do not need a working navigation stack, so they can run in parallel with track A.

### Track A — base, localisation, navigation (critical path)

**CP0 — Environment and assumptions** (0.5 day)

- Confirm assumptions A1–A6 in §1.
- Jetson boots JetPack 6, ROS 2 Humble installed, SSH from the Mac works.
- All four cameras open together at their target resolution; the Mega appears as `/dev/nontok_mega`.
- Pass: `ros2 doctor` clean; four simultaneous camera streams for 10 minutes with no dropouts.

**CP1 — Mega firmware and base control** (1.5 days) — needs CP0

- Sketch built from the probe and lab-assistant code; encoders, PID, ultrasonics, gyro, watchdog, arming.
- Wheels raised: each wheel turns the right way and its encoder counts with the right sign.
- Channel B test, including M4 on A1.
- Counts per revolution checked by turning each wheel 10 turns by hand.
- Pass:
  - Each wheel holds 100, 200 and 300 mm/s within ±10 %.
  - Motors stop within 0.4 s of unplugging the USB cable.
  - Five ultrasonics read a flat board at 300 mm and 1,000 mm within ±20 mm.
  - 30 minutes of driving with no Mega reset and no I²C timeout fault.

**CP2 — ROS bridge, teleoperation and odometry** (1 day) — needs CP1

- `nontok_base` publishes odometry, IMU and five range topics; keyboard teleop drives the robot.
- Calibrate wheel diameter (straight 2 m run), strafe scale (1 m sideways) and gyro scale (10 full turns).
- Pass:
  - Forward 2 m: distance error ≤ 3 %, sideways drift ≤ 50 mm.
  - 360° turn: gyro heading error ≤ 2°.
  - Strafe 1 m: error ≤ 10 % after calibration.
  - 1 m square driven nose-first: returns within 100 mm and 5° of the start.

**CP3 — Cameras and tag detection** (1 day) — needs CP0

- Intrinsic calibration of all four cameras; mounting transforms measured and entered in the URDF.
- Pass:
  - Reprojection error < 0.5 px per camera.
  - A tag at a taped 1.0 m and 2.0 m, straight on: reported distance within 2 % and lateral offset within 20 mm.
  - Detection rates meet §5.3 with all three navigation cameras running, total CPU below 60 %.
  - A tag is still detected while the robot rotates at 0.8 rad/s.

**CP4 — Tag localisation** (2 days) — needs CP2, CP3

- Tags installed and measured; `tags.yaml` written; both EKFs running.
- Pass:
  - Robot placed at 5 taped test poses: reported pose within 50 mm and 3°.
  - Driven by teleop around the full route twice: pose never jumps by more than 150 mm, and ends within 80 mm of truth.
  - Carried to a new place with tags in view: pose recovers within 5 s.

**CP5 — Nav2 point-to-point** (2 days) — needs CP4

- Map drawn; stations entered; Nav2 tuned.
- Pass:
  - 10 of 10 runs between dock approach, pickup approach and each station approach end within 100 mm and 9°.
  - A person stepping 1 m in front stops the robot before contact, and it continues after they leave.
  - A box left on the route is either passed or the robot waits; it never touches it.
  - Controller holds 20 Hz.

**CP6 — Precision docking** (1.5 days) — needs CP5

- Tag servo for stations and the charger.
- Pass:
  - Each station: 10 of 10 approaches from a deliberately offset start (±150 mm, ±10°) end inside ±15 mm, ±20 mm, ±3°.
  - Dock: 9 of 10 approaches end with charging confirmed, including retries.
  - Undock, then re-dock, repeated 5 times in a row.

### Track B — perception (parallel)

**CP7 — Tray detection and classifier** (2 days, needs only the tray and tray camera mounted)

- Item list fixed; dataset captured; model trained and exported.
- Pass:
  - Empty/occupied detection: 50 of 50 placements and removals detected, no trigger from a hand passing over.
  - Station-level accuracy ≥ 95 % on a held-out capture session.
  - 20 live placements of mixed items: at least 19 routed to the right station, none routed wrongly with high confidence.
  - Works under the lab's lights at the pickup point and at one other location.

### Track C — voice (parallel)

**CP8 — Wake word** (1 day, needs only the Jetson and microphone)

- Pass:
  - 18 of 20 detections at 3 m from three different speakers.
  - No more than 1 false trigger in 1 hour of normal lab activity.
  - Keyboard and Bluetooth triggers work as fallbacks.

### Track D — tray mechanism (parallel)

**CP9 — Dump mechanism** (1 day, needs the servo and UBEC)

- Pass:
  - 20 dump cycles with the heaviest demo item: all items leave the tray; no Mega reset or brown-out.
  - Servo supply measured at 5–6 V under load.
  - Tray returns flat enough that the tray camera's empty reference still matches.

### Integration

**CP10 — Full mission** (2 days) — needs CP6, CP7, CP8, CP9

- State machine joins all parts.
- Pass: one complete run for each of the four stations, started by voice, with no human touch between wake word and re-docking.

**CP11 — Reliability and demo readiness** (1–2 days) — needs CP10

- Pass:
  - 8 of 10 consecutive full missions succeed; every failure ends in a safe stop with a logged reason.
  - Each failure branch exercised once on purpose: no item placed, unknown item, blocked route, covered tag at a station, missed dock.
  - A printed start-of-day checklist exists: tags present and flat, lights on, batteries charged, gyro calibrated, empty-tray reference taken.

Estimated effort: about 9–10 working days on the critical path (track A), plus 3–4 days for integration and reliability. With two people working tracks B–D alongside, the whole plan is roughly 12–14 working days.

### If time runs short

Cut in this order, keeping a complete demo at each step:

1. Replace the wake word with the keyboard or Bluetooth trigger.
2. Reduce to two stations.
3. Replace Nav2 with fixed straight-line routes between tags (turn, drive a measured distance, correct on the next tag), reusing the leg-based route idea in `../Nontok/jetson/config/lab_map.yaml`. Tag localisation (CP4) and docking (CP6) are kept; only the planner is dropped.

---

## 9. Safety rules

- A physical cut-off switch on the motor battery line; one person holds it or stands by it during every moving test.
- Motors arm only on command, and disarm on any fault, lost link or `STOP`.
- No reversing or strafing in transit; reversing is limited to the fixed 0.30 m undock move at 0.08 m/s.
- PWM cap stays at 120 until CP1 passes, and transit speed stays at or below 0.25 m/s.
- First test of every new motion is done with the wheels raised.
- Servo and motors never share the Mega's 5 V rail.
- Li-Po handling and charging follow the course's rules; this plan does not change the charging circuit.

---

## 10. Risks

| Risk | Likelihood | Effect | Mitigation |
|---|---|---|---|
| Computer is the older Jetson Nano, not an Orin Nano | Unknown | Stack must change | Settle at CP0 before anything else |
| Tag poses measured inaccurately | Medium | Pose jumps between tags | Two-tag pairs, ±20 mm target, CP4 test poses |
| Motion blur loses tags while turning | Medium | Drift in turns | Fixed short exposure, 0.8 rad/s cap, a tag visible after each turn |
| CPU overload with three detectors plus Nav2 | Medium | Slow control loop | Lower rear and front-R rates, smaller MPPI batch, DWB fallback |
| Ultrasonics miss chair legs and low or overhanging objects | High | Collision | Keep-out areas in the map, clear aisles, low speed |
| M4 encoder direction unavailable (A1 pin) | Medium | Wrong odometry when M4 reverses | Use the command sign for M4; confirmed at CP1 |
| Wake word unreliable for "Nontok" | Medium | Demo cannot start by voice | Three fallbacks in §7.2 |
| Untrained item placed in the tray | High in a live demo | Wrong station | Fixed item list, confidence threshold, "unknown" path |
| Charging coil needs tighter alignment than docking achieves | Medium | No charging | Measure coil tolerance early; add a mechanical guide (funnel) on the dock |
| Servo brown-out resets the Mega | Medium | Mission aborts at the dump | Separate UBEC, tested at CP9 |
| Lighting changes affect tags and the classifier | Medium | Intermittent failures | Matte prints, optional tray LED, test at different times of day |
| Flashing the other task's firmware loses calibration | Low | Lost time | Keep this sketch's calibration in the Jetson's YAML files, not in EEPROM |

---

## 11. Stretch goals (only after CP11)

- Several pickup points, chosen by a spoken bench number after the wake word.
- Spoken feedback ("electrical item, going to station one") through a small speaker.
- Stereo depth from the front pair as an extra obstacle check while stationary.
- Automatic return to the dock on low motor battery (needs the A9 voltage divider).
- TensorRT export of the classifier.

---

## 12. Values to measure (fill in as they are obtained)

| Value | Used in | Measured |
|---|---|---|
| Wheel diameter (mm) | Odometry | |
| Encoder counts per wheel revolution | Odometry | |
| Rotation arm L (mm) | Kinematics | |
| Robot footprint, length × width (mm) | Nav2 | |
| Strafe scale factor | Odometry | |
| Gyro scale factor | Local EKF | 0.991 (27 Oct, re-check) |
| Camera lens field of view and model | Tag range | |
| Camera mounting transforms ×4 | URDF | |
| Ultrasonic mounting positions ×5 | URDF | |
| Front camera baseline (mm) | URDF | |
| Tag poses | `tags.yaml` | |
| Station approach poses and stand-off distances | `stations.yaml` | |
| Charging coil alignment tolerance (mm) | Docking | |
| Charge-sense voltage threshold (mV) | Docking | |
| Tray tipped angle (°) | Dump | |
| Feed-forward constants per wheel | Wheel PID | |

---

## 13. Software implementation plan

This section says exactly what code to write, in what order, and how each piece is tested. Sections 5–7 define *what* the system must do; this section defines *how it is built*.

### 13.1 Ground rules

| Rule | Detail |
|---|---|
| Languages | Firmware in C++ (Arduino framework). All custom ROS nodes in Python 3.10 (`rclpy`). Stock C++ nodes (Nav2, `robot_localization`) are configured, not modified |
| Pure logic is separate from ROS | Every package keeps its maths and decision logic in plain Python modules with no `rclpy` import (for example `kinematics.py`, `protocol.py`, `tag_geometry.py`, `state_machine.py`). The node file only wires topics to those modules. This lets the logic be unit-tested with `pytest` on the Mac, without a robot or ROS |
| No magic numbers in code | Every tunable lives in a YAML file under the package's `config/`, loaded as ROS parameters. Measured values from §12 go in `nontok_description/config/robot.yaml` |
| Units | SI everywhere in ROS. Millimetres and milliradians only on the serial line, converted in one place (`protocol.py`) |
| Time stamps | Every sensor message is stamped with the time the measurement was *taken*, not when it was published. Camera frames are stamped at capture |
| One process per camera | Python's interpreter lock would otherwise make the tag detectors block each other |
| Network | `ROS_LOCALHOST_ONLY=1` on the Jetson so ROS traffic stays off the campus Wi-Fi; remote viewing goes through `foxglove_bridge` (WebSocket, port 8765) |
| Version control | `git init` in this folder; one commit per finished work package; calibration YAML files are committed |
| Editing | Edit on the Mac, sync to the Jetson with `git` or `rsync`, build on the Jetson with `colcon build --symlink-install` |
| Flashing | `pio run -e mega2560 -t upload --upload-port /dev/nontok_mega`. Stop the bridge node first: two programs cannot hold the serial port |

### 13.2 Work packages and order

Each work package (WP) produces code that passes a named checkpoint from §8. Hours are estimates for one person.

| WP | Deliverable | Depends on | Checkpoint | Est. hours |
|---|---|---|---|---|
| WP0 | Jetson environment, udev rules, workspace skeleton, interfaces package | — | CP0 | 4 |
| WP1 | Mega firmware `nontok_ssf_mega` | WP0 | CP1 | 12 |
| WP2 | `nontok_base` bridge, `nontok_description`, teleop, odometry calibration tools | WP1 | CP2 | 8 |
| WP3 | Camera calibration tool, tag generator, `tag_detector` node | WP0 | CP3 | 8 |
| WP4 | `tag_localizer` node, two EKF configs, tag survey check tool | WP2, WP3 | CP4 | 14 |
| WP5 | Map generator, Nav2 configuration, navigation launch | WP4 | CP5 | 14 |
| WP6 | `dock_server` (stations, charger, undock) | WP5 | CP6 | 12 |
| WP7 | Tray monitor, dataset tools, training, classifier node | Tray camera mounted | CP7 | 14 |
| WP8 | Wake-word node and model | WP0 | CP8 | 6 |
| WP9 | Tray servo firmware module and service (part of WP1/WP2 code, tested separately) | WP1 | CP9 | 3 |
| WP10 | Mission state machine, full bringup, autostart | WP6–WP9 | CP10 | 12 |
| WP11 | Logging, failure-branch tests, start-of-day script | WP10 | CP11 | 8 |

Suggested split for a team: one person on WP1 → WP2 → WP5, one on WP3 → WP4 → WP6, one on WP7, one on WP8 → WP9 → WP10.

### 13.3 WP0 — environment and skeleton

Install on the Jetson (JetPack 6, Ubuntu 22.04):

```bash
# ROS 2 Humble base, after adding the ROS apt repository
sudo apt install ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-mppi-controller \
                 ros-humble-robot-localization ros-humble-robot-state-publisher ros-humble-xacro \
                 ros-humble-apriltag-msgs ros-humble-tf-transformations \
                 ros-humble-teleop-twist-keyboard ros-humble-foxglove-bridge
sudo apt install python3-serial python3-opencv python3-yaml v4l-utils libportaudio2
pip3 install --user pupil-apriltags onnxruntime openwakeword sounddevice platformio
```

Checks to make during WP0, because they decide later choices:

- `python3 -c "import pupil_apriltags"` works on the Jetson. If no ARM wheel installs, use OpenCV's built-in detector instead (`cv2.aruco` with `DICT_APRILTAG_36h11`); the detector node hides this behind one function.
- `v4l2-ctl -d <camera> --list-formats-ext` shows an MJPEG mode at 1280×720, and `--list-ctrls` shows a manual exposure control (its name differs between cameras and kernels).

udev rules (`/etc/udev/rules.d/99-nontok.rules`):

- Mega: match `idVendor=1a86`, `idProduct=7523` → symlink `/dev/nontok_mega`.
- Cameras: match each by physical USB port path (`KERNELS==`) → symlinks `/dev/cam_front_left`, `/dev/cam_front_right`, `/dev/cam_rear`, `/dev/cam_tray`. Label the ports on the robot so cables go back in the same place.

Create the ten empty packages from §4 (`ros2 pkg create --build-type ament_python`, except `nontok_interfaces`, which is `ament_cmake`) and confirm `colcon build` succeeds.

`nontok_interfaces` definitions:

```text
msg/BaseStatus.msg      builtin_interfaces/Time stamp
                        bool armed
                        float32 tray_deg
                        float32 charge_v
                        float32 battery_v
                        string fault                 # empty when healthy

msg/TrayState.msg       std_msgs/Header header
                        uint8 EMPTY=0
                        uint8 OCCUPIED=1
                        uint8 MOVING=2               # something is changing (a hand, a rolling item)
                        uint8 state
                        float32 occupied_fraction    # share of tray pixels that differ from the reference
                        bool stable                  # unchanged for stable_s seconds

msg/MissionState.msg    std_msgs/Header header
                        string state                 # S0_DOCKED ... S9_DOCK, FAULT
                        string detail                # station, item, or fault reason

msg/LocalizationStatus.msg
                        std_msgs/Header header
                        uint8 tags_in_view
                        float32 seconds_since_fix
                        float32 metres_since_fix
                        bool healthy

srv/SetTray.srv         float32 angle_deg
                        ---
                        bool success
                        string message

srv/ClassifyItem.srv    ---
                        bool success                 # false = unknown or low confidence
                        string item
                        string station
                        float32 confidence

action/DockToTags.action
                        string station               # key in stations.yaml
                        bool require_charge          # true only for the charging dock
                        ---
                        bool success
                        float32 lateral_error_m
                        float32 distance_error_m
                        float32 yaw_error_rad
                        string message
                        ---
                        string phase
                        float32 lateral_error_m
                        float32 distance_error_m
                        float32 yaw_error_rad
```

Tag detections use the existing `apriltag_msgs/AprilTagDetectionArray`, so no custom message is needed for them.

### 13.4 WP1 — Mega firmware (`firmware/nontok_ssf_mega/`)

Files:

| File | Contents |
|---|---|
| `platformio.ini` | `[env:mega2560]` board `megaatmega2560`, framework `arduino`, `lib_deps = Servo`, monitor speed 115200; `[env:native]` for host unit tests |
| `include/config.h` | Every pin from §3.2 and every constant from §6, nothing else |
| `src/main.cpp` | `setup()`, the 40 ms scheduler, watchdog |
| `src/encoders.{h,cpp}` | Interrupt handlers and atomic count snapshot |
| `src/motors.{h,cpp}` | `motorsWrite(int16_t pwm[4])`, `motorsBrake()`; applies `MOTOR_SIGN` and the PWM limits |
| `src/wheel_control.{h,cpp}` | Mecanum inverse kinematics, acceleration limit, per-wheel PI + feed-forward |
| `src/ultrasonic.{h,cpp}` | One ping per call, round-robin order, result table |
| `src/imu.{h,cpp}` | MPU6050 setup, FIFO drain, bias, health check; adapted from `../Nontok/firmware/hardware_probe_mega` |
| `src/tray.{h,cpp}` | Servo ramp to a target angle |
| `src/power.{h,cpp}` | Charge and battery voltage from A8 and A9 |
| `src/protocol.{h,cpp}` | Line parser and the `ODO`, `RNG`, `STA`, `ERR`, `EVT` writers |
| `test/test_protocol/`, `test/test_control/` | Host tests (PlatformIO Unity) for the parser and the PI controller |

Main loop, every 40 ms:

```text
1. snapshot the four encoder counts (interrupts off for the copy only)
2. measured wheel speed[i] = (count[i] − count 2 cycles ago[i]) × 0.381 mm / 0.080 s
3. if armed and last VEL younger than 300 ms:
       target body velocity → limit acceleration (300 mm/s², 1500 mrad/s²)
       → inverse kinematics → four wheel targets
       pwm[i] = FF_A + FF_B × |target| (signed) + Kp × error + integral
   else: targets = 0, integrators cleared, motors braked
4. write PWM
5. ping one ultrasonic (blocks at most 15 ms)
6. drain the gyro FIFO, average the yaw rate over the cycle
7. send ODO; every 5th cycle send RNG; every 12th cycle send STA
8. step the tray servo ramp
9. read and execute any complete command lines; reset the hardware watchdog
```

Specific implementation points:

- **Encoder interrupt.** Channel A pins map to external interrupts INT3 (D18), INT2 (D19), INT5 (D3), INT4 (D2), attached on `CHANGE`. Each handler reads A and B straight from the port registers (B is PC6, PD7, PL0, PF1) and adds +1 or −1 depending on whether they are equal. A compile-time flag `M4_DIR_FROM_COMMAND` makes wheel 4 use the sign of its last PWM instead.
- **Feed-forward constants.** Found once by a bench routine (`FFTEST` command, wheels raised): hold PWM 60, 90, 120, 150 for 2 s each, print the steady speed, fit a line per wheel.
- **PI starting gains.** Kp = 0.15 PWM per mm/s, Ki = 0.6 PWM per mm/s per second, integral clamped to ±40 PWM. Targets below 15 mm/s are treated as zero, because the motors stall under PWM 60.
- **Parser.** Fixed 48-byte buffer, `strtol` only, no `String` objects. A `!` byte stops the motors the moment it is read, before the line is complete. Unknown or over-long lines produce `ERR` and do not change state.
- **Ultrasonic.** `pulseIn` with a 15,000 µs timeout. Encoder interrupts during the pulse shorten the reading by at most about 2–3 % at full speed, which is acceptable for obstacle detection.
- **Servo.** Attached to D46 only while moving or tipped; detached 0.5 s after returning to rest so it does not buzz or load the supply.
- **Bluetooth.** Serial3 at 9600: `Z` stops and disarms and reports `EVT BTSTOP`; `W` reports `EVT WAKE`. Nothing else is accepted from Bluetooth.
- **Start-up.** Motors disarmed, tray at rest, gyro bias measured once the robot has been still for 1 s, then `READY NONTOK-SSF 1`.

Done when the CP1 tests pass and both host test suites pass.

### 13.5 WP2 — base bridge and robot description

**`nontok_base`**

| File | Contents |
|---|---|
| `nontok_base/protocol.py` | `encode_vel(vx, vy, wz) -> str`, `parse_line(str) -> Odo \| Rng \| Sta \| Err \| Evt \| Ready \| None`; range and field-count validation. Pure Python |
| `nontok_base/kinematics.py` | `wheel_counts_to_body(d_counts, params) -> (dx, dy)`, using the forward kinematics in §5.4 and the calibration scales. Pure Python |
| `nontok_base/clock_sync.py` | Maps Mega `t_ms` to ROS time: offset = smallest (receive time − `t_ms`) seen in the last 10 s. Handles the 32-bit wrap and a Mega reset |
| `nontok_base/base_bridge.py` | The node |
| `nontok_base/fake_mega.py` | A stand-in Mega on a pseudo-terminal: accepts the same commands, integrates `VEL` into encoder counts, emits `ODO`/`RNG`/`STA`. Used by tests and for developing other nodes without the robot |
| `config/base.yaml` | Port, wheel diameter, counts per revolution, rotation arm, `scale_x`, `scale_y`, `gyro_scale`, covariances |
| `test/` | `pytest` for protocol, kinematics, clock sync, and the bridge against `fake_mega` |

Node `base_bridge` interface:

| Direction | Name | Type | Notes |
|---|---|---|---|
| Subscribes | `/cmd_vel` | `geometry_msgs/Twist` | Latest command kept; forwarded as `VEL` at 20 Hz while it is younger than 0.25 s, otherwise zero |
| Publishes | `/wheel/odom` | `nav_msgs/Odometry` | 25 Hz. Twist vx, vy from encoders. Variances start at (0.02 m/s)² for vx and (0.05 m/s)² for vy |
| Publishes | `/imu/data` | `sensor_msgs/Imu` | 25 Hz. Only `angular_velocity.z` is valid (× `gyro_scale`); orientation covariance is set to −1 to mark it unused |
| Publishes | `/range/left45`, `/range/right45`, `/range/front`, `/range/left`, `/range/right` | `sensor_msgs/Range` | 5 Hz each, `min_range` 0.03, `max_range` 1.5, `field_of_view` 0.26 |
| Publishes | `/base/status` | `nontok_interfaces/BaseStatus` | 2 Hz |
| Publishes | `/wake` | `std_msgs/Empty` | On `EVT WAKE` |
| Service | `/base/arm` | `std_srvs/SetBool` | Sends `ARM` / `DISARM`, waits for the reply |
| Service | `/base/stop` | `std_srvs/Trigger` | Sends `STOP` |
| Service | `/base/set_tray` | `nontok_interfaces/SetTray` | Sends `TRAY`, returns after `DONE TRAY` or a 5 s timeout |
| Service | `/base/calibrate_gyro` | `std_srvs/Trigger` | Sends `CAL` |

Range conversion rules:

- A valid echo under 1.5 m is published as measured.
- A valid echo above 1.5 m is published as exactly 1.5 m, which the costmap treats as "clear out to the maximum".
- No echo is published as 1.5 m too, **unless** that sensor read under 0.5 m within the last 0.5 s. In that case nothing is published, because a sudden loss at close range is more likely a slanted surface than an empty space.

Fault handling: no `ODO` for 0.5 s sets `fault`, stops forwarding commands and logs an error. On `READY` (a Mega reset) the bridge re-arms only if the mission asks again.

**`nontok_description`**

- `config/robot.yaml`: footprint, and the position and orientation of each camera and ultrasonic relative to `base_link`.
- `urdf/nontok.urdf.xacro`: reads that file and creates the frames `base_link`, `imu_link`, `cam_front_left_optical`, `cam_front_right_optical`, `cam_rear_optical`, `cam_tray_optical`, `range_left45`, `range_right45`, `range_front`, `range_left`, `range_right`.
- Published by `robot_state_publisher`.

**Tools (`tools/`)**

| Script | Does |
|---|---|
| `calibrate_odometry.py straight` | Drives 2.0 m by encoder; you type the taped distance; prints the corrected wheel diameter |
| `calibrate_odometry.py strafe` | Same for 1.0 m sideways; prints `scale_y` |
| `calibrate_odometry.py spin` | Ten turns by gyro; you type the real residual angle; prints `gyro_scale` |
| `square_test.py` | Drives the CP2 square and prints the closing error |

Done when the CP2 tests pass.

### 13.6 WP3 — cameras and tag detection

| File | Contents |
|---|---|
| `tools/calibrate_camera.py` | Captures 30 views of a 9×6 inner-corner, 25 mm checkerboard; runs `cv2.calibrateCamera`; writes `nontok_localization/config/cameras/<name>.yaml` (matrix, distortion, size, reprojection error) |
| `tools/make_tags.py` | Writes one A4 PDF page per tag ID: 160 mm tag, ID and station name printed below, and a 100 mm scale bar. **Measure the bar after printing**; printers often rescale |
| `nontok_localization/camera.py` | Opens a V4L2 device with OpenCV, sets MJPEG, size, frame rate and fixed exposure; a thread keeps only the newest frame with its capture time |
| `nontok_localization/detector.py` | `detect(gray) -> list[Detection(id, corners[4x2], decision_margin, hamming)]`; wraps `pupil-apriltags` (or the OpenCV fallback) |
| `nontok_localization/tag_detector_node.py` | One instance per camera |
| `config/detectors.yaml` | Per camera: device, size, exposure, rate, `quad_decimate` |

Node `tag_detector` (instances `front_left`, `front_right`, `rear`):

| Direction | Name | Type | Notes |
|---|---|---|---|
| Publishes | `/tags/<camera>/detections` | `apriltag_msgs/AprilTagDetectionArray` | Stamp = capture time; `frame_id` = the camera's optical frame. An empty array is published when nothing is seen, so listeners know the camera is alive |
| Publishes | `/tags/<camera>/debug/compressed` | `sensor_msgs/CompressedImage` | 2 Hz, 640 px wide, tags outlined; only when `debug` is true |
| Parameter | `rate_hz` | float | Can be changed at run time; 0 pauses detection |

Detections with `hamming > 0` or `decision_margin < 50` are dropped in the node. OpenCV is limited to 2 threads per process (`cv2.setNumThreads(2)`).

Done when the CP3 tests pass.

### 13.7 WP4 — tag localisation and state estimation

| File | Contents |
|---|---|
| `nontok_localization/tag_geometry.py` | Loads `tags.yaml`; returns the four 3D corner positions of any tag in the map frame; planar pose helpers. Pure Python + NumPy |
| `nontok_localization/pose_solver.py` | `solve(detections, camera, tag_map) -> Fix \| None`. Pure Python + OpenCV |
| `nontok_localization/tag_localizer_node.py` | The node |
| `config/tags.yaml` | `id: {x, y, z, yaw, size}` for every tag |
| `config/ekf_local.yaml`, `config/ekf_global.yaml` | `robot_localization` settings |
| `tools/verify_tags.py` | With the robot at a known pose, prints for every visible tag the difference between where it is seen and where `tags.yaml` says it is. Catches mis-measured tags before they cause pose jumps |

`pose_solver.solve`, step by step:

1. Keep only detections whose ID is in `tags.yaml`.
2. Build matching lists of image corners and map-frame 3D corners.
3. **Two or more tags:** `cv2.solvePnP` with `SOLVEPNP_SQPNP`, then `cv2.solvePnPRefineLM`. This gives the camera pose in the map directly and uses all corners at once.
4. **One tag:** `cv2.solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE` in the tag's own frame, which returns both possible solutions with their reprojection errors. If the worse error is at least twice the better one, the better solution is trusted fully. Otherwise the solution whose heading is closer to the current estimate is used, and it is marked position-only.
5. Combine with the camera's fixed transform to get `base_link` in the map.
6. **Floor check:** the robot drives on a flat floor, so the solved height must be within ±0.10 m and roll and pitch within ±8°. A solution that fails is discarded. This catches most wrong solutions cheaply.
7. Apply the range, angle and consistency rules from §5.4 and compute the covariance.

Node `tag_localizer`:

| Direction | Name | Type | Notes |
|---|---|---|---|
| Subscribes | `/tags/*/detections` | `AprilTagDetectionArray` | Three topics |
| Subscribes | `/odometry/global` | `nav_msgs/Odometry` | Current estimate, for the consistency rule |
| Publishes | `/tag_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Frame `map`, stamp = image capture time. Position-only fixes carry a yaw variance of 1000 so the filter ignores their heading |
| Publishes | `/localization/status` | `nontok_interfaces/LocalizationStatus` | 2 Hz; `healthy` = a fix within the last 10 s or under 3 m travelled since the last fix |
| Calls | `/ekf_global/set_pose` | `robot_localization/SetPose` | When three consecutive fixes agree with each other but not with the estimate (lost-robot recovery), and when the mission resets the pose on the dock |

EKF configuration (state order is x, y, z, roll, pitch, yaw, then their velocities, then accelerations):

```yaml
# ekf_local.yaml  → publishes odom → base_link and /odometry/local
frequency: 25.0
two_d_mode: true
world_frame: odom
odom0: /wheel/odom          # fuse vx, vy only
odom0_config: [false, false, false,  false, false, false,  true, true, false,  false, false, false,  false, false, false]
imu0: /imu/data             # fuse yaw rate only
imu0_config:  [false, false, false,  false, false, false,  false, false, false,  false, false, true,  false, false, false]

# ekf_global.yaml → publishes map → odom and /odometry/global
frequency: 25.0
two_d_mode: true
world_frame: map
odom0 / imu0: same as above
pose0: /tag_pose            # fuse x, y, yaw
pose0_config: [true, true, false,  false, false, true,  false, false, false,  false, false, false,  false, false, false]
smooth_lagged_data: true    # tag fixes arrive 50–150 ms late; the filter rewinds and replays
history_length: 1.0
```

Tuning method: record a bag of `/wheel/odom`, `/imu/data` and the three detection topics while driving the full route by teleop, then replay it against changed settings. This avoids re-driving the robot for every parameter change.

Done when the CP4 tests pass.

### 13.8 WP5 — map and Nav2

| File | Contents |
|---|---|
| `nontok_navigation/config/lab_layout.yaml` | The lab as a list of rectangles in metres from the dock origin: walls, benches, cabinets, keep-out zones |
| `tools/draw_map.py` | Turns `lab_layout.yaml` into `map.pgm` + `map.yaml` at 0.05 m per cell. The map is regenerated from measurements, never hand-painted, so corrections are one-line edits |
| `config/stations.yaml` | Per station: `approach_pose [x, y, yaw]`, `tag_ids`, `standoff_m`, `lateral_offset_m` |
| `config/nav2_params.yaml` | All Nav2 settings |
| `config/nav_to_pose_no_spin.xml` | Behaviour tree: plan → follow; on failure clear both costmaps, wait 5 s, re-plan; up to 3 rounds. No spin and no back-up nodes |
| `launch/navigation.launch.py` | `map_server` + Nav2's `navigation_launch.py` + lifecycle manager. AMCL is **not** started; localisation comes from WP4 |

Key entries in `nav2_params.yaml` (the rest stay at Nav2 defaults):

```yaml
bt_navigator:
  odom_topic: /odometry/local
  default_nav_to_pose_bt_xml: <path>/nav_to_pose_no_spin.xml

controller_server:
  controller_frequency: 20.0
  FollowPath:
    plugin: nav2_mppi_controller::MPPIController
    motion_model: DiffDrive        # forward + turn only in transit
    time_steps: 40
    model_dt: 0.05
    batch_size: 1000
    vx_max: 0.25
    vx_min: 0.0                    # never reverse
    vy_max: 0.0
    wz_max: 0.8
  goal_checker: {xy_goal_tolerance: 0.10, yaw_goal_tolerance: 0.15}

planner_server:
  GridBased: {plugin: nav2_smac_planner/SmacPlanner2D, tolerance: 0.15}

behavior_server:
  behavior_plugins: [wait]

velocity_smoother:
  max_velocity: [0.25, 0.0, 0.8]
  max_accel: [0.3, 0.0, 1.5]

local_costmap:
  width: 3, height: 3, resolution: 0.05, rolling_window: true
  plugins: [static_layer, range_layer, inflation_layer]
  range_layer:
    plugin: nav2_costmap_2d::RangeSensorLayer
    topics: [/range/left45, /range/right45, /range/front, /range/left, /range/right]
    input_sensor_type: VARIABLE
    clear_on_max_reading: true
    no_readings_timeout: 2.0
    mark_threshold: 0.8
    clear_threshold: 0.2

global_costmap:
  plugins: [static_layer, inflation_layer]   # ultrasonic marks stay local only
```

Tuning order: (1) footprint and inflation until planned paths keep sensible clearance; (2) MPPI with obstacles absent until paths are tracked smoothly; (3) range layer thresholds with a person and a box as obstacles; (4) batch size down if the 20 Hz loop is missed.

Done when the CP5 tests pass.

### 13.9 WP6 — docking server

| File | Contents |
|---|---|
| `nontok_docking/dock_controller.py` | The control law and phase logic. Pure Python |
| `nontok_docking/dock_server_node.py` | Action server |
| `config/docking.yaml` | Gains, speed limits, tolerances, timeouts |

Node `dock_server`:

| Direction | Name | Type |
|---|---|---|
| Action server | `/dock_to_tags` | `nontok_interfaces/DockToTags` |
| Service | `/undock` | `std_srvs/Trigger` — reverse 0.30 m at 0.08 m/s by odometry, then stop |
| Subscribes | `/tags/front_left/detections`, `/tags/front_right/detections` | Tag corners |
| Subscribes | `/odometry/local`, `/range/front`, `/base/status` | |
| Publishes | `/cmd_vel` | Only while an action or undock is running; Nav2 is idle then |

How it controls:

1. Each time the station's tag pair is detected, solve the robot's pose relative to the pair (same `pose_solver`, with the pair's own geometry as the map).
2. Convert that into a **target pose in the `odom` frame**, using the odometry at the image's capture time.
3. A 20 Hz loop drives the robot toward that target using odometry alone. New detections only move the target.

The robot therefore moves smoothly even though detections arrive at about 10 Hz and 100 ms late, and it keeps going briefly if a tag drops out.

Phases:

| Phase | Action | Leaves when |
|---|---|---|
| ACQUIRE | Stand still | Tag pair seen in 5 consecutive frames (or 3 s timeout → fail) |
| APPROACH | Drive to a point `standoff + 0.15 m` out, squared up; limits 0.08 m/s, 0.3 rad/s | Within 30 mm and 3° |
| FINAL | Drive straight in to `standoff`; limit 0.04 m/s | Within tolerance |
| BLIND | If the pair leaves the image inside 0.30 m: finish on odometry, cross-checked by the front ultrasonic | Within tolerance |
| SETTLE | Zero velocity; check the error for 0.5 s | Still within tolerance → success |
| CHARGE (dock only) | Wait for `charge_v` above threshold for 1 s | Success, or back off 0.15 m and repeat from APPROACH (3 tries) |

Control law per axis: velocity = gain × error, clipped to the limit. Starting gains: forward 0.8 /s, sideways 1.0 /s, heading 1.5 /s. Errors are smoothed with a first-order filter (factor 0.5) before use.

Safety inside the server: no tag for 0.3 s outside the BLIND phase → zero velocity; no tag for 2 s → fail; front ultrasonic under 0.05 m → stop and fail; total timeout 30 s.

Done when the CP6 tests pass.

### 13.10 WP7 — tray monitor and classifier

| File | Contents |
|---|---|
| `nontok_perception/tray_logic.py` | Difference image, occupancy, stability, item crop. Pure Python + OpenCV |
| `nontok_perception/classifier.py` | ONNX Runtime wrapper: `classify(crop) -> (item, confidence)` |
| `nontok_perception/tray_node.py` | The node |
| `config/tray.yaml` | Tray region polygon, thresholds, model path |
| `config/items.yaml` | `item: station` lookup, for example `resistor: electrical` |
| `training/capture_tray_dataset.py` | Runs on the robot. Shows the tray camera; a key press saves the **item crop produced by `tray_logic`** plus the full frame, under `dataset/<session>/<item>/` |
| `training/split_by_session.py` | Whole sessions go to train or validation, never mixed |
| `training/train_cls.py` | `yolo classify train model=yolo11n-cls.pt imgsz=224 epochs=60`, with rotation up to 180°, both flips, and colour jitter |
| `training/export_onnx.py` | `yolo export format=onnx imgsz=224` and a parity check |
| `training/eval_station_accuracy.py` | Confusion matrix by item and by station on the held-out session |

Tray logic, with starting thresholds:

1. Convert to grey, blur 5×5, mask to the tray region.
2. `changed` = pixels differing from the empty reference by more than 25 grey levels.
3. `occupied_fraction` = changed pixels ÷ tray pixels. Occupied above 0.02; empty again below 0.01 (the gap prevents flicker).
4. `stable` = fewer than 0.5 % of tray pixels changed between consecutive frames for 2 s.
5. **Item crop** = bounding box of the changed pixels, grown by 20 %, padded to a square, at least 96 px a side, resized to 224×224.

The crop matters: a resistor fills only a few percent of the tray, so shrinking the whole tray image to 224 px would leave too few pixels to recognise it. Training images and live images must come from the same crop function, which is why the capture tool imports `tray_logic`.

Classifier input: RGB, values 0–1, 224×224, channels first. The parity check in `export_onnx.py` runs 20 validation images through both Ultralytics and the ONNX path and requires the same top answer on all 20; this catches any preprocessing mismatch.

Node `tray_node`:

| Direction | Name | Type | Notes |
|---|---|---|---|
| Publishes | `/tray/state` | `nontok_interfaces/TrayState` | 5 Hz while enabled |
| Service | `/tray/enable` | `std_srvs/SetBool` | Opens or releases the camera; off while driving to save CPU |
| Service | `/tray/capture_reference` | `std_srvs/Trigger` | Averages 5 frames as the empty reference |
| Service | `/tray/classify` | `nontok_interfaces/ClassifyItem` | 5 frames, majority vote, mean confidence ≥ 0.80, then `items.yaml` lookup |

Done when the CP7 tests pass.

### 13.11 WP8 — wake word

| File | Contents |
|---|---|
| `nontok_voice/wake_logic.py` | Score smoothing and trigger decision. Pure Python |
| `nontok_voice/wake_node.py` | Audio capture and the node |
| `config/voice.yaml` | Model path, threshold, audio device, sample rate |
| `models/nontok.onnx` | Trained with openWakeWord's automatic training notebook |
| `tools/eval_wake.py` | Scores a folder of positive recordings and a long background recording; prints detection rate and false triggers per hour for a range of thresholds |
| `tools/trigger.py` | Publishes `/wake` from the keyboard |

Node `wake_node`:

- Audio: `sounddevice` input stream, 16 kHz mono, 1,280-sample blocks (80 ms).
- Each block goes to the model, which returns a score from 0 to 1.
- Trigger when the score is at least 0.5 in 2 of the last 3 blocks; then ignore audio for 3 s.
- Publishes `std_msgs/Empty` on `/wake`.
- Service `/voice/enable` (`std_srvs/SetBool`): the mission enables it only in S0.

Threshold is chosen from `eval_wake.py` output as the lowest value that keeps false triggers under one per hour.

Done when the CP8 tests pass.

### 13.12 WP10 — mission and bringup

| File | Contents |
|---|---|
| `nontok_mission/state_machine.py` | States, transitions, timeouts and retry counts from §7.1. Talks to the robot only through a small `RobotApi` interface. Pure Python |
| `nontok_mission/robot_api.py` | The real `RobotApi`: Nav2 Simple Commander for navigation, action client for docking, service clients for tray, classifier, arm, voice |
| `nontok_mission/mission_node.py` | 10 Hz tick; publishes `/mission/state`; writes the log |
| `config/mission.yaml` | Timeouts, retry limits, tray angle and hold time |
| `test/test_state_machine.py` | Drives the state machine with a fake `RobotApi`: one test for the normal path per station and one per failure branch in §7.1 |

`RobotApi` methods (each returns immediately; long actions are polled):

```text
start_navigation(station) / navigation_result() -> RUNNING | OK | FAILED
start_dock(station, require_charge) / dock_result()
undock() / undock_result()
set_tray(angle_deg) -> bool
tray_state() -> EMPTY | OCCUPIED | MOVING, stable
capture_tray_reference(), classify() -> (ok, item, station, confidence)
arm(bool), stop(), reset_pose_to_dock()
enable_wake(bool), wake_requested() -> bool
localization_healthy() -> bool
show(text)                      # OLED line
```

Timeouts and limits (`mission.yaml`):

| Item | Value |
|---|---|
| Navigation leg | 120 s, 2 attempts |
| Dock or align | 30 s per attempt; 2 attempts at stations, 3 at the charger |
| Wait for item | 120 s |
| Classification retries | 2 |
| Dump hold | 1.5 s; 1 repeat if the tray is not empty |
| Localisation unhealthy while moving | Stop, wait 5 s for a fix, then FAULT |

Logging: every state change, classification result, docking error and fault is appended as one JSON line to `~/nontok_logs/<date>.jsonl`. CP11's success count is read from this file.

**`nontok_bringup`** launch files:

| Launch file | Starts |
|---|---|
| `base.launch.py` | `base_bridge`, `robot_state_publisher` |
| `teleop.launch.py` | `base.launch.py` + keyboard teleop |
| `localization.launch.py` | Three `tag_detector` nodes, `tag_localizer`, both EKFs |
| `navigation.launch.py` | Map server and Nav2 |
| `perception.launch.py` | `tray_node` |
| `voice.launch.py` | `wake_node` |
| `nontok.launch.py` | Everything above + `dock_server`, `mission_node`, `foxglove_bridge` |
| `fake.launch.py` | Same as `nontok.launch.py` but with `fake_mega` instead of the serial port, for testing off the robot |

Autostart: a `systemd` unit `nontok.service` runs `nontok.launch.py` at boot, restarts on failure, and is enabled only once CP10 has passed.

Done when the CP10 tests pass.

### 13.13 WP11 — reliability tooling

| File | Does |
|---|---|
| `tools/preflight.py` | Start-of-day check, prints pass or fail per line: Mega replies to `PING`; all four cameras deliver frames; each ultrasonic returns a reading; gyro bias is fresh; battery voltage above limit; dock tags visible from the dock; pose within 50 mm of the origin |
| `tools/mission_report.py` | Summarises the day's log: missions started, succeeded, failed by reason, mean duration |
| `tools/record_bag.sh` | Records the standard topic set for later replay |

### 13.14 Interface summary

Topics, in one place:

| Topic | Type | From → to |
|---|---|---|
| `/cmd_vel` | `Twist` | Nav2 or `dock_server` or teleop → `base_bridge` |
| `/wheel/odom` | `Odometry` | `base_bridge` → both EKFs |
| `/imu/data` | `Imu` | `base_bridge` → both EKFs |
| `/range/*` (5) | `Range` | `base_bridge` → local costmap, `dock_server` |
| `/base/status` | `BaseStatus` | `base_bridge` → `dock_server`, mission |
| `/tags/<camera>/detections` (3) | `AprilTagDetectionArray` | `tag_detector` → `tag_localizer`, `dock_server` |
| `/tag_pose` | `PoseWithCovarianceStamped` | `tag_localizer` → global EKF |
| `/odometry/local` | `Odometry` | Local EKF → Nav2, `dock_server` |
| `/odometry/global` | `Odometry` | Global EKF → `tag_localizer`, mission |
| `/localization/status` | `LocalizationStatus` | `tag_localizer` → mission |
| `/tray/state` | `TrayState` | `tray_node` → mission |
| `/wake` | `Empty` | `wake_node`, `base_bridge`, `trigger.py` → mission |
| `/mission/state` | `MissionState` | Mission → Foxglove, log |

Transforms:

| Transform | Published by |
|---|---|
| `map → odom` | Global EKF |
| `odom → base_link` | Local EKF |
| `base_link →` cameras, ultrasonics, IMU | `robot_state_publisher` (static) |

Only one node may publish `/cmd_vel` at a time. The mission guarantees this by never starting a docking action while a navigation goal is active, and teleop is never launched together with the mission.

### 13.15 Test strategy

| Level | What | Where it runs | When |
|---|---|---|---|
| Unit | Protocol, kinematics, clock sync, tag geometry, pose solver (synthetic projected corners with known answers), dock controller, tray logic (saved images), wake logic, state machine | `pytest` on the Mac or Jetson | Every commit |
| Firmware host | Parser and PI controller | `pio test -e native` | Every firmware change |
| Integration without the robot | Full launch against `fake_mega`; mission runs end to end with scripted tray and wake events | Jetson, `fake.launch.py` | Before each robot session |
| Replay | Localisation and EKF settings against recorded bags | Jetson | During WP4 tuning |
| Robot | The checkpoint tests in §8 | Lab | At the end of each WP |

A work package is finished only when its unit tests pass, its checkpoint passes on the robot, its measured values are written into §12 and the YAML files, and the result is committed.

---

## 14. Implementation status and changes from the plan

First implementation pass, 12 November 2026, done without the robot. Where this section and sections 1–13 disagree, this section describes the code.

### 14.1 Status

| Work package | Code | Verified on a computer | Still needs hardware |
|---|---|---|---|
| WP0 skeleton, interfaces | Ten packages, messages, udev rules, systemd unit | — | Jetson setup, `colcon build` |
| WP1 firmware | Complete | Compiles for the Mega (2 variants); 14 host tests | Flash; all of CP1 |
| WP2 base bridge | Complete, plus simulated Mega | 34 tests, including over a pseudo serial port | CP2 calibration |
| WP3 detection | Detector node, tag sheet generator, camera calibration tool | Detector finds rendered tags; sheet checked at 160 mm | Cameras, calibration |
| WP4 localisation | Solver, gate, localiser node, EKF config, `verify_tags` | 22 tests on synthetic views | Real tags; EKF tuning |
| WP5 navigation | Map generator, stations, Nav2 parameters, behaviour trees | 6 tests; Nav2 drives the simulated robot between all stations | Tuning on the real floor, obstacles |
| WP6 docking | Controller and action server | 15 tests on a simulated robot with camera delay and noise | CP6 |
| WP7 perception | Tray logic, classifier wrapper, node, training scripts | 11 tests on synthetic images | Item list, dataset, model |
| WP8 voice | Trigger logic, node, threshold tool | 3 tests | Microphone, model |
| WP10 mission | State machine, ROS wrapper, launch files | 23 tests: every station and every failure branch | CP10 |
| WP11 tools | `preflight.py`, `mission_report.py`, `record_bag.sh` | Report logic checked | — |

Test count: 130 Python tests and 14 firmware host tests, all passing.

Second pass, same day, in a ROS 2 Humble container (`docker/Dockerfile`): `colcon build` passes for all ten packages, the full launch starts with `fake:=true`, and a simulated mission (`tools/sim_in_docker.sh <item>`) completed for all four stations in 108–165 s. The simulation has no camera images, wheel slip or obstacles.

Third pass, 13 November 2026: docking in those runs ended 3–30 mm from the true dock, and in repeated trials up to 34 mm and 20° off while reporting success. Cause: inside the dock server the sensor subscriptions shared an executor with the long-running action and were starved, so tag measurements reached the controller 1–3 s late and odometry arrived at 3–8 Hz. After the changes in 14.2, 10 of 10 simulated dockings from random starts ended within 5 mm and 0.1° of the true dock, and two full missions docked within 5 mm.

Fourth pass, 16 November 2026, on the team's Jetson (section 14.4): the image built for the laptop runs there unchanged, `colcon build` takes 1 min 40 s, the tests pass, and after the changes marked "Jetson" in 14.2 a simulated mission completed for all four stations in 89–96 s, each alignment within 10 mm, with no late or rejected tag measurements.

Fifth pass, 16 November 2026, wake word and item recognition (no robot movement):

- **Wake word.** `models/nontok.onnx` now exists. It was trained (`training/wake/`, three steps) on about 5,600 synthetic "Nontok" clips in roughly 900 voices, including Thai and Cantonese system voices, mixed with lab sound recorded on the Jetson, against 370 h of public speech features. On held-out data: 93 % of calls detected in training-style voices, 80 % in an unseen voice engine, 84–98 % in the streaming pipeline when the caller is at least 5 dB louder than the room, and 2–3 false triggers per hour of unseen speech. On the Jetson it runs from an XWF camera's microphone at 40 % of one core, only while listening is enabled, and gave no trigger in a minute of lab sound. **No real person has said the word to it yet.** Try it with `ros2 run nontok_voice listen`; recordings of the team saying it should be added to `data/pos` and the model retrained.
- **Item recognition.** No dataset exists, so a second classifier was added: `ClipZeroShot` picks the item whose written description (in `items.yaml`) fits the picture best, using the CLIP ViT-B/32 image model (`models/clip_vision.onnx`, 350 MB, not in git; `training/build_item_prompts.py` fetches it and writes `items_clip.npz`). Scores are added up per station and frames vote on the station. On the Jetson it takes 0.8 s per image. On reference photos of the item types every one went to the right station, and views of the room are rejected as "not an item". The tray node was run on the Jetson with a real camera (reference, state at 5 Hz, "unknown" on an empty view). **It has not seen a real item in the real tray.** A trained `items.onnx`, when present, is used instead.

Sixth pass, night of 16–17 November 2026, preparation for the first day on the robot:

- **Tag detection was unaffordable and is now cheap.** Two real detectors (1280x720) used 200 % and 160 % of a core. Causes: every frame was decoded to colour 30 times a second (19 ms each), the loop polled, and the OpenCV detector was in use. Now the camera thread keeps the JPEG and only the frames that are searched are decoded, straight to grey (8 ms); the AprilTag library is used (`dt-apriltags`, which has an arm64 build; `pupil-apriltags` will not build on the Jetson) with one thread and `quad_decimate` 3; rates are 6, 6 and 3 images per second. Measured: 36 % of a core at 6 Hz, 19 % at 3 Hz. No tag has been in front of a camera yet, so range and accuracy at `quad_decimate` 3 are unverified.
- **Rear and tray cameras take turns.** The tray node sets the rear detector's `paused` parameter, which makes it release its camera, before opening the tray camera, and clears it afterwards. Tested on the Jetson with the two cameras that cannot stream together.
- **The robot speaks.** `speech_node` plays recorded phrases (`nontok_voice/sounds/`, made by `training/make_speech.py` from `config/phrases.yaml`): it answers the call, asks for the item, says what it is and where it is going, and says when it is back or needs help. With the zero-shot item model the item named can be a look-alike from the same station. No speaker has been connected yet; the node picks the first USB speaker.
- **First power-on tool.** `ros2 run nontok_base wheel_check` spins the wheels slowly in three patterns with the robot on blocks and checks each encoder's sign and count. Run against the simulated Mega only.
- **On the Jetson:** `tools/jetson.sh` (build, shell, run, launch, cameras, stop). `docs/tags_160mm.pdf` holds the twelve labelled dock and station tags for printing at 100 % scale.
- A simulated mission on the Jetson with all of this passed (92 s) but logged one watchdog trip and 43 missed controller cycles, more than the earlier runs; watch the load.

Seventh pass, early 17 November 2026, live dashboard:

- **Dashboard.** `nontok_viz/viz_node` serves one web page on port 8080 of the Jetson: the map with the robot, its trail, the planned path, ultrasonic beams and which tags are in view; mission state, display text, motors, faults, position, speed, localisation, tags per camera, ranges, tray, wake word, computer load, temperature and memory; an event list (state changes, wake word, phrases spoken); warnings from all nodes; and a small picture from each camera. It is part of `nontok.launch.py` (`dashboard:=false` to leave it out). On a laptop: `tools/mac_viz.sh` opens an SSH tunnel and the browser; `tools/mac_viz.sh sim` first starts a simulated mission on the Jetson. Verified from the Mac against the Jetson with a simulated robot. No real camera picture has been through it yet.
- It costs the Jetson about 7 % of a core while nobody looks and about 40 % while the page is open: the fast topics and the camera pictures are only subscribed to and produced while it is being watched (`viz/watching`).
- **The Jetson Nano is over its CPU budget.** With the dashboard and speech nodes added, the simulated mission on the Jetson failed in most runs (tag alignment starved: odometry reaching the dock server at 4-10 Hz and up to 2 s old); with both switched off (`dashboard:=false speech:=false`) it passed two of two, with retries. Run-to-run variation is large. Tried and rejected because they made timing worse or no better: Nav2 in one process, a slimmer Nav2 launch, CycloneDDS, the dock server listening only while docking, the global filter at 10 Hz. Kept: dashboard, voice and tray nodes run at low priority (`nice 10`, tag localiser `nice 5`), and the tray node no longer loads the 600 MB vision model in simulation. What has not been tried: stopping the desktop session on the Jetson, and moving the handlers of the fast topics out of Python.

### 14.2 Changes from sections 1–13

| Topic | Plan | As built | Why |
|---|---|---|---|
| Tag detection message | `apriltag_msgs/AprilTagDetectionArray` | Own `nontok_interfaces/TagDetectionArray` | No dependence on a message layout that could not be checked here; corner order is fixed in the definition |
| Pose from tags (13.7) | OpenCV `solvePnP` (SQPNP, IPPE_SQUARE), then a floor check | A direct fit of the three unknowns x, y, heading with the robot held on the floor | The OpenCV square-tag solver returned wrong answers on perfectly symmetric views in testing. The three-number fit is better conditioned and cannot return a floating or leaning robot |
| Single distant tag | Position from the image, heading ignored | Position from the tag's distance and bearing, combined with the current heading estimate | The image alone fixes the viewing angle poorly; each degree of error became centimetres of sideways error |
| Corner order of the detector library | Assumed from documentation | The detector checks itself at start-up against a generated tag and adapts | Removes a silent failure if the two supported libraries differ |
| URDF | `xacro` file | Generated in Python from `robot.yaml` | Testable without ROS; one fewer dependency |
| Station positions | `approach_pose` typed into `stations.yaml` | Computed from the station's two tags in `tags.yaml` | A position is written down once only |
| Leaving a station | Only the dock had an undock move | A 0.30 m reverse after the pickup point and after every dump as well (states `S4_LEAVE_PICKUP`, `S7_LEAVE_STATION`) | Nav2 is forward-only and cannot start with the robot's nose at a wall |
| Disarmed motors | Braked | Braked for 1 s, then released | The robot can be pushed by hand when disarmed |
| Arming and the watchdog | Not specified | Watchdog runs whenever armed and disarms; the bridge sends `VEL` at 20 Hz while armed, zeros included | A crashed Jetson leaves the robot disarmed, not merely stopped |
| Dock charge check | Always | Behind `dock_requires_charge` (mission) and `CHARGE_SENSE_FITTED` (firmware), both off | The sense divider does not exist yet |
| Leaving FAULT | Charger voltage | Charger voltage, or `ros2 service call /mission/reset` | Works before charge sensing exists |
| Fake mode | Full launch against the fake Mega | Same, with a simulated tray, `fake_item`, and a `fake_tags` node that publishes the detections each camera would make from the simulated Mega's position | Lets localisation, docking and the whole mission run without hardware |
| Nav2 motion (5.6) | Forward and turn only (`DiffDrive`) | `Omni` with a sidestep of at most 0.08 m/s; still no reversing | Forward-only, the simulated robot stalled 0.25 m beside a goal and the mission faulted |
| Nav2 goal tolerance | 0.10 m, 0.15 rad | 0.20 m, 0.25 rad | Tag alignment starts from the Nav2 goal and removes the rest |
| Nav2 behaviour trees | One custom tree | Two: the navigator will not start unless the through-poses tree also loads, and the stock one needs `spin` | Found at first launch |
| Watchdog trip while driving | Base stays disarmed | The bridge re-arms 0.3 s after a trip, at most 3 times a minute, and never after a stop, disarm, Bluetooth stop or Mega restart | A 300 ms stall of the computer otherwise ended the mission with a navigation time-out; it happened in simulation |
| Filter reset from tags | Whenever needed | At most once every 2 s | At start-up the localiser reset the filter on every frame until its first output arrived |
| Dock server inputs (13.9) | Subscriptions on the action server's node | Odometry, ranges, status and tag detections on a second node with its own thread; `/tf` on its own thread; tag queue depth 1 | Sharing the executor delayed measurements by seconds; this was the cause of the docking misses |
| Late tag measurements | Used | Dropped when older than `max_measurement_age_s` (0.4 s); the controller's tag time-outs use the image time, not the arrival time | Steering on an old view makes the robot overshoot and weave |
| End of docking | Stop on entering the tolerance | Stop on entering half the tolerance (`stop_fraction`), then check against the full tolerance | The robot stopped at the edge of the 10 mm band |
| Docking log | — | One line per docking: result, errors, number of fixes, oldest measurement, rejection counts, odometry rate | Shows this fault at a glance if it returns on the Jetson |
| Velocity stream to the Mega | Executor timer | Own thread | The timer was held up for more than 300 ms now and then, tripping the watchdog |
| Nav2 controller (Jetson) | MPPI | DWB behind the rotation shim, 10 Hz, same limits (0.25 m/s forward, 0.08 m/s sidestep, no reversing) | The packaged MPPI controller dies with an illegal instruction on the Nano's Cortex-A57. DWB alone sat still when the goal was behind the robot; the shim turns it first |
| Ultrasonic silence (Jetson) | Range layer `no_readings_timeout` 2 s | Layer timeout off; the base bridge sends zero velocity and reports `RANGES_LOST` when no `RNG` line arrives for 1 s | In this Nav2 release one gap over the timeout (a slow start is enough) leaves the local costmap "not current" for good and the controller waits for ever, with no error. Seen on the Jetson and reproduced on a laptop limited to 1.5 CPUs |
| Pose fit (Jetson) | OpenCV projection in every step, numeric derivatives | Corners undistorted once, plain pinhole arithmetic, analytic derivatives, looser stopping rule | 18–35 ms per image on the Nano; now about 13 ms before the stopping rule was loosened, same answers |
| Tag fixes used | Every image | At most `max_fix_rate_hz` (5) per camera | The fit is the localiser's main cost and the filter gains nothing from more |
| Camera mounting lookup (Jetson) | Each node listens to `/tf` | A throw-away node finds the mountings once and is destroyed (`CameraFinder`) | Unpacking every transform cost each Python node a large share of a Nano core. Do not remove a `/tf` subscription from a spinning node: that left nodes hung |
| Foxglove bridge | Always started | Only with `viewer:=true` | CPU |
| ROS-dependent tools | `tools/` | Console scripts inside their packages: `calibrate_odometry` (includes the square test), `make_tags`, `calibrate_camera`, `verify_tags`, `draw_map`, `eval_wake` | They import package code |
| M4 encoder direction | Decide at CP1 | Defaults to "from the command" (`M4_DIR_FROM_COMMAND 1`) | A1 is reported broken; safest default |

### 14.3 Known gaps and first things to check on hardware

- `ENC_SIGN` in the firmware `config.h` is a guess; check each wheel at CP1.
- The feed-forward constants and PWM cap of 120 limit top speed; the 0.25 m/s transit speed may need the cap raised after CP1.
- `nav2_params.yaml` now runs in simulation, but its tuning has only seen a perfect robot in an empty map. One simulated run logged a `Failed to make progress` that the retry recovered from.
- On the Jetson keep numpy below 2 (`pip3 install 'numpy<2' onnxruntime`). `pupil-apriltags` had no arm64 package in the container; the detector then falls back to OpenCV, which gives no decision margin.
- The tray difference detector needs contrast between item and tray floor: a grey item on a grey tray is not seen. Choose the tray colour with the item list in mind.
- The simulated tags are ideal corner positions with 0.3 px noise; no image has gone through the detector node.
- The other nodes that use a multi-threaded executor (mission, tray, base bridge services) have not been checked for the delays found in the dock server. On the Jetson, read the docking log line: the oldest tag message should stay near 0.1 s and odometry near 25 Hz.
- CPU on the Jetson Nano is the tightest resource. With simulated tags the launch uses about half of the four cores; real MJPG decoding and tag detection for three cameras are still to be added and measured. Stop the desktop session for real runs (`sudo systemctl isolate multi-user.target`).
- `tools/sim_in_docker.sh` accepts `NONTOK_DOCKER_ARGS="--cpus 1.5"`; that limit reproduces Jetson-like timing faults on a laptop and is worth using before every push to the robot.

### 14.4 The Jetson as found (16 November 2026)

| Item | Value |
|---|---|
| Board | Jetson Nano Developer Kit, 4 GB, Cortex-A57 x4 |
| System | JetPack 4.6 (L4T R32.6.1), Ubuntu 18.04, kernel 4.9.253-tegra, Python 3.6 |
| ROS 2 | Humble inside Docker only: the `nontok-dev` image from `docker/Dockerfile` (Ubuntu 22.04, arm64). Build it on an arm64 laptop and copy it over with `docker save nontok-dev \| gzip \| ssh ... docker load`; Docker 20.10.2 on the Jetson runs it. Use `--privileged --network host` and pass the cameras and the Mega through for real runs |
| Project | `~/nontok` (copied with rsync; build inside the container) |
| GPU in the container | Not set up. `onnxruntime` runs on the CPU |
| Fan | Automatic; it starts near 50 °C. `echo 255 \| sudo tee /sys/devices/pwm-fan/target_pwm` forces it on |

Cameras: two "1080P USB Camera" (`2bdf:0289`) and two "XWF 1080P PC Camera" (`1bcf:2285`), all MJPG, each fine alone.

- All four USB ports share one USB 2.0 bus and each camera reserves most of it, so the stock driver streams only one camera at a time.
- A patched `uvcvideo` is installed on the Jetson (source and build in `~/uvc-patched`, stock module saved as `~/uvc-patched/uvcvideo.ko.stock`, option `mjpg_bandwidth=800` in `/etc/modprobe.d/nontok-uvcvideo.conf`). It caps the bandwidth reserved for compressed streams. With it three cameras stream together: 640x480 at 20–28 fps, 1280x720 at 14–19 fps, no bad frames.
- The two `2bdf` cameras can never stream together on this bus: they offer a single bandwidth setting and two of them exceed the limit.
- **Fitted (2026-11-21): three cameras, no rear.** XWF pair = front (as mounted 23 Nov: `/dev/cam_front_left` port 1-2.1, `/dev/cam_front_right` port 1-2.3, checked by plugging in one at a time; 1-2.4 has a small hub with the USB speaker and the Mega), one `2bdf` = tray (`/dev/cam_tray`, port 1-2.2). All three stream together (MJPEG 640x480: 30 + ~24 + ~19 fps, no "Not enough bandwidth", no OC alarms), so the tray node pauses no detector (`pause_detector: ""`). The rear detector is not launched; back_in stations would need a rear camera again.
- The cameras of a pair have identical names and serial numbers; address them by USB port (`/dev/v4l/by-path/...usb-0:2.N:1.0-video-index0`) and keep each in its port.
- A JetPack update that replaces the kernel modules removes the patched driver.

