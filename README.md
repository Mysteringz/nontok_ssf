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

## Layout

```text
firmware/nontok_ssf_mega/   Mega firmware (PlatformIO)
ros2_ws/src/                ten ROS 2 packages; each keeps its logic in plain Python modules with tests
training/                   dataset capture, training, export and scoring for the item classifier
tools/                      preflight check, mission log report, bag recording, simulated mission
docker/                     ROS 2 Humble image for building and simulating on a laptop
```
