# DiffBot Raspberry Pi 4 (4GB) — ROS 2 Humble Python

Converted from uploaded `Ros2Scripts.zip` and informed by `turtlebot3_core.zip`.

## Status
**Starter implementation; NOT tested on your hardware.** Motors are disabled by default. Includes GPIO DC motor control, quadrature encoder counting, PID wheel speed control, odometry/TF, joint states, optional ultrasonic node, URDF, optional SLAM Toolbox and Nav2 launch hooks. Includes RPLIDAR support via `rplidar_ros`. Nav2 configuration and map/localization must be completed before autonomous use.

## Install (Ubuntu 24.04 64-bit, ROS 2 Humble installed)

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-gpiozero python3-lgpio ros-humble-robot-state-publisher ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rplidar-ros
# from this workspace directory:
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch diffbot_pi robot.launch.py
```

GPIO backend availability and permissions depend on OS image. Test `gpiozero` on your Pi. On Pi 4, gpiozero can use lgpio. Avoid mixing competing GPIO processes.

## Mandatory setup BEFORE enabling motors

1. Review `src/diffbot_pi/config/robot.yaml`: pin mapping is taken from uploaded motor and encoder scripts. Confirm wiring and H-bridge signal conventions; **GPIO pins cannot power DC motors directly**. Use separate motor supply and common ground.
2. Ultrasonic original pins conflict with motor PWM GPIO4 and encoder GPIO17. Suggested GPIO22/26 are **not existing wiring**. Keep sonar disabled until rewired. 5V ultrasonic ECHO must be level shifted to 3.3V before Pi GPIO.
3. Measure wheel radius, wheel separation, and wheel encoder ticks per **meter of ground travel**. Default geometry and 5000 ticks/m are placeholders.
4. Confirm encoder A/B phase and wheel direction. Set `left_encoder_sign`, `right_encoder_sign`, `left_motor_sign`, `right_motor_sign` to +/-1 as needed. Incorrect sign can make PID accelerate unexpectedly.
5. Lift wheels clear of the floor. Set `enable_motors: true`, start with small commands, confirm `/odom` increases forward. Implement a physical emergency stop and motor-driver power cutoff; software timeout is not a substitute.
6. Tune `feedforward`, `kp`, `ki` at low speed. Encoder polling in Linux/Python can miss pulses at high rates; for high-resolution/high-speed encoders use dedicated counter hardware or microcontroller.

## Useful commands

```bash
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}'
ros2 launch diffbot_pi robot.launch.py sonar:=true
```

The command timeout is 0.5 s, so a single velocity command stops quickly. For sustained manual motion use a teleop publisher.

## SLAM / Nav2 / LiDAR

- **RPLIDAR** is started by default (`lidar:=false` to disable) via `rplidar_ros`, publishing `sensor_msgs/LaserScan` on `/scan` in frame `laser`.
  1. `sudo apt install ros-humble-rplidar-ros`
  2. Set `serial_baudrate` in the `rplidar` section of `config/robot.yaml` for your model (A1/A2M8: 115200, A2M12/A3/S1: 256000, C1: 460800, S2: 1000000).
  3. Stable device name: `sudo cp src/diffbot_pi/udev/99-rplidar.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`, then replug. This creates `/dev/rplidar`. Without it, set `serial_port: /dev/ttyUSB0` and add your user to `dialout`.
  4. Measure the lidar mount and update `laser_joint` origin in `urdf/robot.urdf`. In RViz (Fixed Frame `base_link`), an object in front of the robot must appear at +x; if it appears behind, set the `laser_joint` yaw to `3.14159`.
  5. Test: `ros2 launch diffbot_pi robot.launch.py` then `ros2 topic hz /scan`.
- SLAM uses `config/slam.yaml` (`base_frame: base_link`, matching this robot's TF tree).
- SLAM launch hook: `ros2 launch diffbot_pi robot.launch.py slam:=true` **only after LiDAR and calibrated odometry**. SLAM Toolbox publishes `map -> odom`.
- Nav2 hook: `ros2 launch diffbot_pi robot.launch.py nav2:=true` **not turnkey**. Nav2 needs a properly configured costmap/footprint, planner/controller, robot limits, and either SLAM (`map -> odom`) or a saved map + localization. Nav2's standard default configuration may not match this robot. Do not enable navigation until validated.
- Run RViz2 on a separate workstation. URDF dimensions/laser pose are illustrative and must be measured.
- Avoid two nodes publishing the same `odom -> base_link` TF. This package publishes it directly; don't enable another odometry TF broadcaster simultaneously.

## Known limitations

- Pi Linux is not hard real-time; quadrature encoder counting via gpiozero may lose counts under load.
- PID is basic PI with bounded integral, not production-tuned; velocity is derived from encoder counts at 20 Hz.
- `enable_motors=false` mode publishes stationary encoder/odometry data for software bring-up, not a physics simulation.
- Battery, IMU, magnetometer, RC100 and OpenCR-only firmware functions from TurtleBot3 are **not** implemented without hardware details.
- No automatic ultrasonic obstacle fusion into Nav2 costmaps is configured.
