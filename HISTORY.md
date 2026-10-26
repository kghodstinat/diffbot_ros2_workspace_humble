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

## 2026-10-25 — Arduino + ros2_control study package
- Built a separate workspace `diffbot_arduino_ws` (zip in `~/Downloads/diffbot_arduino_ws.zip`); this repo left unchanged.
- Contents: Arduino firmware (encoder interrupts, PID @ 30 Hz, ros_arduino_bridge-compatible protocol, 2 s auto-stop); `diffbot_hardware` C++ ros2_control SystemInterface plugin (POSIX serial); `diffbot_bringup` (xacro URDF with `<ros2_control>`, `controllers.yaml` for diff_drive_controller, robot-specific `nav2_params.yaml`, launch files for Pi and computer incl. AMCL on a saved map); `diffbot_sonar` (logger bug fixed).
- Not built or tested yet; C++ plugin only reviewed by hand.

### How the Arduino + ros2_control version works (explained)
Three layers, each only knows its neighbor:
- **diff_drive_controller (ROS, Pi, 30 Hz)** — robot math only, never touches hardware.
  - Inverse kinematics: `v_l = v − ω·L/2`, `v_r = v + ω·L/2`, `ω_wheel = v_wheel / r` → writes wheel velocity **command interfaces** (rad/s).
  - Forward kinematics: reads wheel position **state interfaces** → `d = r(Δθ_l+Δθ_r)/2`, `Δθ = r(Δθ_r−Δθ_l)/L` → `/odom` + TF `odom→base_link`.
- **C++ plugin `DiffBotArduinoHardware` (Pi)** — translator between interfaces and serial.
  - Lifecycle: `on_init` (read URDF params) → `on_configure` (open serial, wait 2 s for Arduino reset) → `on_activate` (send `r`) → loop `read()`/`write()` → `on_deactivate` (send `m 0 0`).
  - `read()`: send `e` → counts → `position = counts·2π/counts_per_rev`, `velocity = Δposition/dt`.
  - `write()`: `counts_per_frame = ω·counts_per_rev/(2π·30)` → send `m L R`.
  - Interfaces are shared pointers to the plugin's own variables (`left_.command`, `left_.position`, …).
- **Arduino Nano firmware** — timing-critical work.
  - Encoder ISRs with a 16-entry quadrature lookup table (4× decoding, no missed pulses).
  - Incremental PID at 30 Hz: `pwm += (Kp·err − Kd·Δmeasured + I)/Ko`; PWM on L298N IN pins.
  - Auto-stop after 2 s without `m`/`o`.
- Why split: Arduino gives guaranteed microsecond timing; the Pi does math and translation at 30 Hz where small delays are fine. Swapping the Nano (ESP32) or the robot (simulator) only needs a new plugin.

### How the launch file connects ROS to the Arduino
`robot.launch.py` never opens the port itself:
1. Runs `xacro` → `robot_description`, passing `arduino_port` (default `/dev/arduino`).
2. URDF `<ros2_control>` block names the plugin (`diffbot_hardware/DiffBotArduinoHardware`) and params (`device`, `baud_rate` 57600, `enc_counts_per_rev`, `loop_rate`).
3. Starts `ros2_control_node` with `robot_description` + `controllers.yaml`; remaps `/diff_cont/cmd_vel_unstamped → /cmd_vel`, `/diff_cont/odom → /odom`.
4. `ros2_control_node` loads the plugin via pluginlib (`diffbot_hardware.xml` → `libdiffbot_hardware.so`); `on_configure()` opens the serial port.
5. Spawners start `joint_broad` and `diff_cont`; then read/write runs at 30 Hz.
- `/dev/arduino` comes from `udev/99-arduino.rules`; otherwise use `arduino_port:=/dev/ttyUSB0`.

## Open items
- Run the spin/straight test; calibrate `ticks_per_meter`, `wheel_separation`, signs, lidar yaw.
- Confirm motor + motor-driver models (decides Nano vs ESP32/Pico and firmware tweaks).
- Fix `sonar.py` logger bug; decide `enable_motors` default; wire saved-map localization; write Nav2 params.
- Commit these notes (`HISTORY.md`, `.claude/skills/diffbot/SKILL.md`).
