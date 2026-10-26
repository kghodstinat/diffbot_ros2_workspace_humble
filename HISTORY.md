# DiffBot — Project History

Reference details (commands, troubleshooting, wiring) live in `.claude/skills/diffbot/SKILL.md`.

## 2026-10-24 — RPLIDAR added
- Added RPLIDAR support to `diffbot_pi` using `rplidar_ros` (`rplidar_node`):
  - `config/robot.yaml`: new `rplidar` section (port `/dev/rplidar`, 115200 baud, frame `laser`).
  - `launch/robot.launch.py`: rplidar node with `lidar:=true|false` (default true).
  - `launch/lidar.launch.py`: new — lidar + robot_state_publisher only, for testing.
  - `udev/99-rplidar.rules`: stable `/dev/rplidar` symlink.
  - `config/slam.yaml`: new — SLAM Toolbox params with `base_frame: base_link` (default `base_footprint` doesn't exist on this robot; SLAM would fail without this).
  - `package.xml` depends on `rplidar_ros`; `setup.py` installs `udev/`; URDF laser visual; README2 LiDAR section.
- `apt install ros-humble-rplidar-ros` failed with 404 → cause: expired ROS apt key (`EXPKEYSIG`). Fixed on the Pi with the `ros2-apt-source` package.

## 2026-10-25 — First run, SLAM, split architecture
- Full launch on Pi worked: lidar healthy at 115200 baud (fw 1.29, hw rev 7, 10 Hz, 12 m), SLAM produced a map, Nav2 became active.
- RViz on the computer (Parallels VM) showed libGL/GLSL errors → `LIBGL_ALWAYS_SOFTWARE=1`; remaining GLSL error is a harmless RViz2 bug. Map visible.
- Map quality was low. Diagnosis: placeholder odometry calibration, unverified encoder/motor signs, encoder pulses missed under Pi CPU load, possible lidar yaw, coarse SLAM thresholds. Defined the spin/straight test for calibration.
- **Decision:** Pi runs only the robot (`robot.launch.py`); SLAM, Nav2, RViz, teleop run on the computer. `slam.yaml` copied to the computer (`~/slam.yaml`). Clock sync with chrony; `ROS_DOMAIN_ID=30` on both.
- Added Pi `~/.bashrc` sourcing of ROS + workspace.
- Gamepad teleop via `teleop_twist_joy` on the computer; hit the same expired-key 404 on the computer, same fix. Two joystick devices seen (`js0`, `js1`) — pick the controller with `joy_enumerate_devices` and `joy_dev:=<id>`.
- Commands collected in `run.txt`.

## 2026-10-25 — Architecture decisions for the future
- C++ rewrite of `base_controller` on the Pi rejected as the fix: Linux, not Python, causes missed encoder pulses.
- **Chosen direction:** Arduino Nano with `ros_arduino_bridge` firmware (encoders, PID, PWM) connected to the Pi over USB serial; Pi runs `ros2_control` + `diff_drive_controller` + `diffdrive_arduino`. Then add an IMU (BNO055/BNO085) + `robot_localization`, and a robot-specific Nav2 params file.
- Nano wiring plan drafted (assumes L298N-style driver; driver model not yet confirmed).

## Open items
- Run the spin/straight test; calibrate `ticks_per_meter`, `wheel_separation`, signs, lidar yaw.
- Confirm motor + motor-driver models (decides Nano vs ESP32/Pico and firmware tweaks).
- Fix `sonar.py` logger bug; decide `enable_motors` default; wire saved-map localization; write Nav2 params.
- Commit these notes (`HISTORY.md`, `.claude/skills/diffbot/SKILL.md`).
