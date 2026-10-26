---
name: diffbot
description: Reference for this DiffBot robot (Raspberry Pi 4 + RPLIDAR + ROS 2 Humble) — architecture, which commands run on the Pi vs the computer, setup, troubleshooting, calibration, and the planned Arduino Nano / ros2_control upgrade. Use for any question about running, configuring, debugging, or extending this robot.
---

# DiffBot — ROS 2 Humble reference

See `HISTORY.md` at repo root for the dated log of decisions.

## Hardware (current)
- Raspberry Pi 4, Ubuntu 22.04 (Jammy), ROS 2 Humble. Hostname `ubuntu`, user `kaveh`.
- Computer: Ubuntu 22.04 arm64 VM in Parallels on a Mac (hostname `ubuntu22`).
- 2 DC motors with quadrature encoders, H-bridge driven directly from Pi GPIO (gpiozero).
- RPLIDAR on USB (CP210x). Works at **115200 baud**, firmware 1.29, hardware rev 7, Sensitivity mode, 10 Hz, 12 m range (A1/A2M8 class).
- Optional HC-SR04 sonar (GPIO 22/26, disabled by default).
- Xbox-style gamepad on the computer (shows as `/dev/input/js0` + `js1`; one is a Parallels virtual device).

## Package layout (`src/diffbot_pi`)
- `diffbot_pi/base_controller.py` — motors, encoders, PI speed control, `/odom`, TF `odom -> base_link`, `/joint_states`.
- `diffbot_pi/sonar.py` — `/sonar_front` Range.
- `config/robot.yaml` — params for `base_controller`, `sonar`, `rplidar`.
- `config/slam.yaml` — SLAM Toolbox params; `base_frame: base_link` (robot has **no** `base_footprint`).
- `launch/robot.launch.py` — base_controller, rplidar (`lidar:=true` default), robot_state_publisher, optional `sonar:=`, `slam:=`, `nav2:=`.
- `launch/lidar.launch.py` — lidar + robot_state_publisher only (for testing).
- `udev/99-rplidar.rules` — creates `/dev/rplidar`.
- `urdf/robot.urdf` — `base_link`, wheels, `laser` (x=0.08, z=0.10, yaw 0 — unverified), `ultrasonic_front_link`.

## Architecture (recommended split)
| Pi (robot) | Computer |
|---|---|
| base_controller, rplidar driver, robot_state_publisher | SLAM Toolbox, Nav2, RViz, teleop (keyboard/joy) |

Reason: keeps Pi CPU free so Python encoder counting misses fewer pulses. `slam:=true nav2:=true` on the Pi runs them **on the Pi** (avoid).

## Commands

### Pi
```bash
ros2 launch diffbot_pi robot.launch.py          # robot only (lidar on)
ros2 launch diffbot_pi lidar.launch.py          # lidar test only
ros2 topic hz /scan                             # ~10 Hz expected
ros2 service call /stop_motor std_srvs/srv/Empty  # stop lidar motor
```

### Computer (each in its own terminal)
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/slam.yaml
ros2 launch nav2_bringup navigation_launch.py
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
ros2 run teleop_twist_keyboard teleop_twist_keyboard                 # keyboard
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_dev:=<id>   # gamepad, hold LB
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
- Close teleop before using **Nav2 Goal** (both publish `/cmd_vel`).
- `slam.yaml` on the computer is a **copy** from the Pi — recopy after edits:
  `scp kaveh@<pi-ip>:~/diffbot_ros2_workspace_humble/src/diffbot_pi/config/slam.yaml ~/slam.yaml`
- `navigation_launch.py` comes from apt (`ros-humble-nav2-bringup`); uses Nav2 default params. Custom params later: `params_file:=$HOME/nav2_params.yaml`.
- Joystick: find the ID with `ros2 run joy joy_enumerate_devices`; ROS ID may differ from `js0/js1` numbering.

## One-time setup
Both machines:
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
sudo apt install chrony        # clock sync, required when nodes span machines
```
Pi `~/.bashrc`: `source /opt/ros/humble/setup.bash` and `source ~/diffbot_ros2_workspace_humble/install/setup.bash`.
Computer `~/.bashrc`: `source /opt/ros/humble/setup.bash`, `export LIBGL_ALWAYS_SOFTWARE=1`.

Pi packages: `ros-humble-rplidar-ros` (+ GPIO libs, robot-state-publisher).
Computer packages: `ros-humble-desktop ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard ros-humble-joy ros-humble-teleop-twist-joy joystick`.

udev (Pi): `sudo cp src/diffbot_pi/udev/99-rplidar.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`, replug.

## Troubleshooting
- **apt 404 / `EXPKEYSIG F42ED6FBAB17C654`**: ROS apt key expired (hit on both Pi and computer). Fix:
  ```bash
  sudo rm -f /etc/apt/sources.list.d/ros2*.list /usr/share/keyrings/ros-archive-keyring.gpg
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb && sudo apt update && sudo apt upgrade
  ```
  If still failing: `grep -rl packages.ros.org /etc/apt/sources.list /etc/apt/sources.list.d/` and remove all but `ros2.sources`.
- **RViz libGL / GLSL errors in Parallels VM**: use `LIBGL_ALWAYS_SOFTWARE=1`. The remaining `indexed_8bit_image ... GLSL link result` error is a harmless RViz2 Humble bug — the map still renders.
- **SLAM "Message Filter dropping message ... queue is full"** at startup: normal, stops after a second. `minimum laser range ... 0.2 m` warning: harmless.
- **Computer doesn't see topics**: same Wi-Fi, same `ROS_DOMAIN_ID`, firewall (`sudo ufw disable` to test), guest networks may block multicast.
- **Lidar errors**: `operation time out` / `cannot retrieve health code` = wrong baud; `cannot bind to the specified serial port` = permissions/port.

## Map quality — diagnosis and calibration
Likely causes, most to least likely:
1. `ticks_per_meter: 5000`, `wheel_separation: 0.245` are **placeholders**.
2. Encoder/motor `*_sign` values unverified.
3. Python/gpiozero misses encoder pulses under CPU load.
4. Lidar yaw in `laser_joint` may need `3.14159`.
5. SLAM `minimum_travel_distance/heading: 0.5` coarse for a small robot (try 0.2/0.2; `resolution: 0.025`).

**Spin/straight test** (Pi: `robot.launch.py`, no SLAM; RViz Fixed Frame `odom`, LaserScan Decay Time 20):
- Rotate slowly 360°: walls must stay still. Rotate opposite way → lidar yaw or encoder sign wrong. Smear same way → `wheel_separation` wrong (odom turns > 360° → increase it).
- Drive straight 1 m (tape-measured): new `ticks_per_meter = 5000 × (1.0 / odom_x)`.
- Forward must increase `/odom` x; left turn must increase yaw.

## Known issues / TODO
- `sonar.py` uses printf-style `%s` args in rclpy logger calls (lines ~52-55, 83) → TypeError on error paths.
- `enable_motors: true` in yaml and code defaults, though README says disabled by default.
- `map` launch arg declared but unused — no localization on a saved map yet (each launch maps from scratch).
- No robot-specific Nav2 params file (using nav2_bringup defaults).
- README2 said Ubuntu 24.04; actual OS is 22.04 (required for Humble).

## Upgrade roadmap (decided direction)
"Real-time" is a Linux problem, not a Python one — rewriting `base_controller` in C++ on the Pi does not fix missed encoder pulses.
1. **Arduino Nano** (chosen over Pi GPIO and over Pico for ready-made firmware + 5 V tolerance) runs `ros_arduino_bridge` firmware (Articulated Robotics fork): encoder interrupts, PID, PWM. Not ROS, not rosbridge, not micro-ROS.
2. Pi runs **ros2_control + diff_drive_controller + diffdrive_arduino** plugin over USB serial (57600 baud) — replaces `base_controller.py`. Same `/cmd_vel`, `/odom`, TF.
   Serial protocol: `e` → `left right` ticks; `m L R` → speeds (ticks/loop); `o L R` → raw PWM; `r` → reset.
3. IMU (BNO055/BNO085 on Pi I²C) + `robot_localization` EKF.
4. Robot-specific Nav2 params (footprint, speeds); optionally sonar via Nav2 range sensor layer.

**Study package built (2026-10-25):** `~/Downloads/diffbot_arduino_ws.zip` — firmware, `diffbot_hardware` (C++ plugin), `diffbot_bringup` (xacro, controllers.yaml, nav2_params.yaml, robot/lidar/navigation launch), `diffbot_sonar`. Untested. How it works + launch→Arduino chain: see HISTORY.md.
Pi: `ros2 launch diffbot_bringup robot.launch.py` · Computer: `ros2 launch diffbot_bringup navigation.launch.py [slam:=false map:=~/my_map.yaml]`.

Pi-only alternative if staying on GPIO: kernel `rotary-encoder` dtoverlay in `/boot/firmware/config.txt`, hardware-PWM pins (GPIO 12/13/18/19; current PWM pins 4/25 are software PWM).

### Nano wiring (L298N-style driver; ros_arduino_bridge defaults)
| Signal | Old Pi GPIO | Nano |
|---|---|---|
| Left IN fwd / bwd | 15 / 14 | D10 / D6 |
| Left ENABLE | 4 | D13 (held HIGH, or ENA jumper) |
| Right IN fwd / bwd | 24 / 23 | D9 / D5 |
| Right ENABLE | 25 | D12 (held HIGH, or ENB jumper) |
| Left encoder A / B | 17 / 27 | D2 / D3 |
| Right encoder A / B | 5 / 6 | A4 / A5 |
| Encoder power / GND | Pi | Nano 5V / GND |

Common ground: Nano + driver + motor battery. Motors powered from battery via driver, never from Nano. Nano powered by Pi USB. Motor driver model still unconfirmed (TB6612 etc. needs firmware tweak).
