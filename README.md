# Nontok SSF — self-return robot

Design, spec sheet and checkpoints: [PLAN.md](PLAN.md).

## Status (13 November 2026)

Written without access to the robot. The workspace builds under ROS 2 Humble and the whole
mission runs against a simulated robot, on a laptop and on the team's Jetson Nano (in Docker).
Nothing has run with the real cameras, the Mega or the motors yet.

| Part | State |
|---|---|
| Mega firmware `firmware/nontok_ssf_mega` | Compiles for the Mega 2560 (with and without OLED). 14 host tests pass. Never flashed. |
| Logic of every ROS package | 119 tests pass, on the Mac and in the ROS 2 Humble container (Python 3.10, OpenCV 4.5.4). |
| ROS build and launch | `colcon build` passes for all ten packages. `nontok.launch.py fake:=true` starts every node, Nav2 and both filters. |
| Simulated mission | Dock, pickup, classify, station, dump and re-dock completed for all four stations (105-165 s each). |
| Simulated docking | 10 of 10 from random starts end within 5 mm and 0.1 degrees of the true dock, after fixing a fault that delayed tag measurements by seconds (PLAN.md 14.1). |
| Classifier model, wake-word model | Not trained: need the tray camera, the item list and voice recordings. |
| `tags.yaml`, `lab_layout.yaml`, `robot.yaml`, camera calibrations | Placeholders: need measurements. |

What the simulation does not cover: real cameras and tag detection in images, wheel slip,
obstacles (the simulated ultrasonics see nothing), the classifier, the wake word, the firmware.

## Check it yourself

```bash
cd ros2_ws && python3 -m pytest                      # needs numpy, opencv, pyyaml, pyserial, pytest
cd firmware/nontok_ssf_mega && pio run && pio test -e native
tools/sim_in_docker.sh bolt                          # needs Docker: builds, launches, runs one simulated mission
NONTOK_DOCKER_ARGS="--cpus 1.5" tools/sim_in_docker.sh bolt   # the same on a slow computer, like the Jetson Nano
```

## First session with hardware

Follow PLAN.md section 8 from CP0. In short:

To watch the robot live from the Mac (map, state, sensors, cameras, events) in a browser:

```
tools/mac_viz.sh              # connect to the running robot and open the dashboard
tools/mac_viz.sh sim bolt     # or: start a simulated mission on the Jetson and watch that
tools/mac_viz.sh stop
```

Demo without the robot (progress reports):

```
tools/demo.sh bolt            # on the Mac: simulated mission in Docker, dashboard at localhost:8081
tools/demo.sh busy bolt       # the same in a room with two tables (on the map) and two walking people (not on it)
tools/demo.sh voice           # then: say "Nontok" to the Mac (or press Enter); it answers on the Mac's speakers
tools/demo.sh stop
tools/jetson.sh camdemo       # on the Jetson: real cameras, wake word and dashboard only - no motors;
                              # watch with tools/mac_viz.sh
```

On the Jetson run without the desktop (`sudo systemctl set-default multi-user.target`; back with
`graphical.target`) and call the robot no sooner than a minute after starting the software.

On the Jetson everything runs through `tools/jetson.sh` (from `~/nontok`):

```
tools/jetson.sh build                                  # colcon build in the container
tools/jetson.sh cameras                                # which camera is on which USB port
tools/jetson.sh run ros2 run nontok_base wheel_check   # FIRST power-on: robot on blocks
tools/jetson.sh run ros2 run nontok_voice listen --model src/nontok_voice/models/nontok.onnx --device XWF
tools/jetson.sh launch fake:=true fake_item:=bolt      # whole mission, simulated
tools/jetson.sh launch                                 # the real robot
```

1. Jetson (an original Nano on Ubuntu 18.04, so ROS 2 runs in Docker only; PLAN.md 14.4): load the
   `nontok-dev` image, install `ros2_ws/src/nontok_bringup/system/99-nontok.rules` on the host,
   then inside the container `cd ros2_ws && colcon build --symlink-install`.
   Keep numpy below 2 (`pip3 install 'numpy<2' onnxruntime`): the system OpenCV breaks with numpy 2.
   `pupil-apriltags` had no ready-made arm64 package in the container; if pip cannot install it on the
   Jetson the detector uses OpenCV's AprilTag support instead.
2. Without the robot: `ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=bolt`,
   then `python3 tools/sim_mission.py` in a second terminal.
3. Mega: wheels raised, flash, then work through CP1 (`ENC_SIGN`, `M4_DIR_FROM_COMMAND`, `FFTEST`).
4. `ros2 launch nontok_bringup teleop.launch.py`, then `ros2 run nontok_base calibrate_odometry straight|strafe|spin|square`.

## Switching the Mega firmware

The robot's Mega holds one program at a time: this repo's firmware (`nontok_ssf`, used by
`tools/wheel_bench.py` and the ROS bridge) or the teammate's `HARDWARE_PROBE_2` from the Nontok repo
(used by its probe dashboard). Only the tools of the firmware on the Mega work.

1. On the Mac, after committing: `tools/build_mega_hex.sh [path/to/Nontok]` builds both, names each hex
   after its commit and copies them to the Jetson (refuses uncommitted firmware changes; `DIRTY=1` allows them).
2. On the Jetson: `tools/mega_firmware.sh probe`, `ssf` or `which`. Writes and verifies (probe ~30 s,
   ssf ~7 s), then confirms the boot line. Nothing moves; the port must be free.

The probe dashboard runs on the Mac (Python 3.7 or newer), so with the Mega's USB on the Jetson it
connects over Bluetooth (HC-06), or move the USB cable to the Mac.

## Bench and floor tests (`tools/wheel_bench.py`, on the Jetson)

Talks to our firmware on `/dev/nontok_mega` (Python 3.6 is enough). Every mode that drives asks for
a typed confirmation first, and every exit (end, Ctrl-C, error, lost SSH) sends `!` and DISARM.
Raw logs go to `~/Documents/David/wheel_bench/`. Results so far: [docs/cp1_results.md](docs/cp1_results.md).

```
cd ~/nontok/tools
python3 wheel_bench.py watch                      # never arms: live encoder counts for the hand test
python3 wheel_bench.py ff                         # wheels RAISED: PWM 30-120 both ways, fits FF_A/FF_B
python3 wheel_bench.py hold                       # wheels RAISED: speed hold 100/200/300, strafe, turn
python3 wheel_bench.py drive --distance 1000      # FLOOR: straight run by encoder distance, then stop
python3 wheel_bench.py route --route "F1500 R90 U200 D"   # FLOOR: legs, ultrasonic approach, tray dump
```

Route steps: `F`/`B` mm straight, `S` mm strafe (+ left), `L`/`R` degrees turn (gyro), `U` mm creep
until the front ultrasonic reads that distance, `D` dump (130°, hold 1.5 s, back), `W` seconds wait.
Floor modes stop for an obstacle seen twice in a row (front < 250 mm, a 45° sensor < 150 mm), a
stall, or a timeout.

Wiring of every device on the shield: [docs/wiring.md](docs/wiring.md).

To use the teammate's probe dashboard on the Mac while the Mega is plugged into the Jetson, flash
their firmware (`mega_firmware.sh probe`), then on the Mac run `python3 tools/serial_bridge.py` and
start their dashboard on the port it prints (details in the script).

Printing for camera calibration and tags: `docs/checkerboard_9x6_25mm_A4.pdf`
(`tools/make_checkerboard.py`) and `docs/tags_160mm_labelled.pdf`, at 100 % scale.

## Layout

```text
firmware/nontok_ssf_mega/   Mega firmware (PlatformIO)
ros2_ws/src/                ten ROS 2 packages; each keeps its logic in plain Python modules with tests
training/                   dataset capture, training, export and scoring for the item classifier
  video/                    item classifier from laptop videos: record, extract, train, test
tools/                      preflight check, mission log report, bag recording, simulated mission
docker/                     ROS 2 Humble image for building and simulating on a laptop
```
