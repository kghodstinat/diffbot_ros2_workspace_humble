"""Combined encoder, wheel-speed, odometry, joint-state, and watchdog node.

Hardware: GPIO motor driver with two direction inputs and one PWM enable per
wheel. Encoder steps must be calibrated for the wheel output shaft. A Raspberry
Pi is not a real-time system.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64
from tf2_ros import TransformBroadcaster


DEFAULT_PARAMETERS = {
    # Right motor H-bridge pins (IN1, IN2, PWM)
    "right_a": 24,
    "right_b": 23,
    "right_pwm": 25,

    # Left motor H-bridge pins (IN1, IN2, PWM)
    "left_a": 15,
    "left_b": 14,
    "left_pwm": 4,

    # Left wheel encoder GPIO pins
    "left_enc_a": 17,
    "left_enc_b": 27,

    # Right wheel encoder GPIO pins
    "right_enc_a": 5,
    "right_enc_b": 6,

    # Robot geometry and wheel calibration
    "ticks_per_meter": 5000.0,
    "wheel_separation": 0.245,
    "wheel_radius": 0.033,

    # Controller timing and limits
    "control_hz": 20.0,
    "cmd_timeout": 0.5,
    "max_wheel_speed": 0.35,

    # PID and feedforward tuning
    "kp": 0.8,
    "ki": 0.2,
    "feedforward": 0.65,

    # Motor and encoder direction signs
    "left_encoder_sign": 1,
    "right_encoder_sign": 1,
    "left_motor_sign": 1,
    "right_motor_sign": 1,

    # Safety / startup flag
    "enable_motors": True,
}


def quaternion_yaw(theta):
    """Return a quaternion representing a rotation around the Z axis."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(theta / 2),
        w=math.cos(theta / 2),
    )


class MotorHardware:
    """Hardware-specific GPIO and encoder access for the diff-drive base."""

    def __init__(self, params):
        self.params = params
        self.left_motor = None
        self.right_motor = None
        self.left_encoder = None
        self.right_encoder = None

        if not self.params["enable_motors"]:
            return

        from gpiozero import Motor, RotaryEncoder

        self.left_motor = Motor(
            forward=self.params["left_a"],
            backward=self.params["left_b"],
            enable=self.params["left_pwm"],
            pwm=True,
        )
        self.right_motor = Motor(
            forward=self.params["right_a"],
            backward=self.params["right_b"],
            enable=self.params["right_pwm"],
            pwm=True,
        )
        self.left_encoder = RotaryEncoder(
            a=self.params["left_enc_a"],
            b=self.params["left_enc_b"],
            max_steps=0,
        )
        self.right_encoder = RotaryEncoder(
            a=self.params["right_enc_a"],
            b=self.params["right_enc_b"],
            max_steps=0,
        )

    def read_ticks(self):
        if self.left_encoder is None or self.right_encoder is None:
            return 0, 0

        left_ticks = (
            int(self.left_encoder.steps) * self.params["left_encoder_sign"]
        )
        right_ticks = (
            int(self.right_encoder.steps) * self.params["right_encoder_sign"]
        )
        return left_ticks, right_ticks

    def drive_wheels(self, left_speed, right_speed):
        if self.left_motor is None or self.right_motor is None:
            return

        self._drive_motor(self.left_motor, left_speed, self.params["left_motor_sign"])
        self._drive_motor(self.right_motor, right_speed, self.params["right_motor_sign"])

    def _drive_motor(self, motor, speed, sign):
        speed = max(-1.0, min(1.0, speed * sign))
        if abs(speed) < 0.001:
            motor.stop()
        elif speed > 0:
            motor.forward(speed)
        else:
            motor.backward(-speed)

    def stop(self):
        for motor in (self.left_motor, self.right_motor):
            if motor is not None:
                motor.stop()
                motor.close()

        for encoder in (self.left_encoder, self.right_encoder):
            if encoder is not None:
                encoder.close()


class BaseController(Node):
    def __init__(self):
        super().__init__("base_controller")

        self._declare_parameters()
        self._validate_parameters()
        self._initialize_state()
        self._initialize_hardware()
        self._create_ros_interfaces()

        self.get_logger().warn(
            "MOTORS "
            + (
                "ENABLED: lift wheels for initial tests"
                if self.p["enable_motors"]
                else "DISABLED: simulation of stationary encoders only"
            )
        )

    def _declare_parameters(self):
        for key, value in DEFAULT_PARAMETERS.items():
            self.declare_parameter(key, value)
        self.p = {
            key: self.get_parameter(key).value
            for key in DEFAULT_PARAMETERS
        }

    def _validate_parameters(self):
        if (
            self.p["ticks_per_meter"] <= 0
            or self.p["wheel_separation"] <= 0
            or self.p["wheel_radius"] <= 0
        ):
            raise ValueError("Geometry/ticks_per_meter must be positive")

    def _initialize_state(self):
        self.left_ticks = self.right_ticks = 0
        self.last_left = self.last_right = 0
        self.x = self.y = self.theta = 0.0
        self.target_l = self.target_r = 0.0
        self.integral_l = self.integral_r = 0.0
        self.last_command = time.monotonic() - 1000
        self.last_update = time.monotonic()

    def _initialize_hardware(self):
        self.hw = MotorHardware(self.p)
        self.left_motor = self.hw.left_motor
        self.right_motor = self.hw.right_motor
        self.left_encoder = self.hw.left_encoder
        self.right_encoder = self.hw.right_encoder

    def _create_ros_interfaces(self):
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.left_pub = self.create_publisher(Int64, "lwheel", 10)
        self.right_pub = self.create_publisher(Int64, "rwheel", 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self.command, 10)
        self.create_timer(1 / self.p["control_hz"], self.update)

    def command(self, msg):
        velocity = float(msg.linear.x)
        angular_velocity = float(msg.angular.z)
        half_separation = self.p["wheel_separation"] / 2
        limit = self.p["max_wheel_speed"]

        self.target_l = max(
            -limit,
            min(limit, velocity - angular_velocity * half_separation),
        )
        self.target_r = max(
            -limit,
            min(limit, velocity + angular_velocity * half_separation),
        )
        self.last_command = time.monotonic()

    def drive(self, motor, speed, sign):
        if self.hw is not None:
            self.hw._drive_motor(motor, speed, sign)

    def drive_wheels(self, left_speed, right_speed):
        if self.hw is not None:
            self.hw.drive_wheels(left_speed, right_speed)

    def pid(self, target, measured, integral, dt):
        if abs(target) < 1e-4:
            return 0.0, 0.0

        error = target - measured
        integral = max(-1.0, min(1.0, integral + error * dt))
        effort = (
            self.p["feedforward"] * target / self.p["max_wheel_speed"]
            + self.p["kp"] * error
            + self.p["ki"] * integral
        )
        return max(-1.0, min(1.0, effort)), integral

    def update(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        if dt <= 0:
            return

        if now - self.last_command > self.p["cmd_timeout"]:
            self.target_l = self.target_r = 0.0

        if self.hw is not None:
            left_ticks, right_ticks = self.hw.read_ticks()
            if self.hw.left_encoder is not None:
                self.left_ticks = left_ticks
                self.right_ticks = right_ticks

        dl = (self.left_ticks - self.last_left) / self.p["ticks_per_meter"]
        dr = (self.right_ticks - self.last_right) / self.p["ticks_per_meter"]
        self.last_left = self.left_ticks
        self.last_right = self.right_ticks
        vl = dl / dt
        vr = dr / dt

        if self.hw is not None and self.hw.left_motor is not None:
            ul, self.integral_l = self.pid(
                self.target_l, vl, self.integral_l, dt
            )
            ur, self.integral_r = self.pid(
                self.target_r, vr, self.integral_r, dt
            )
            self.hw.drive_wheels(ul, ur)

        self._update_odometry(dl, dr, vl, vr)
        self._publish_state(vl, vr)

    def _update_odometry(self, dl, dr, vl, vr):
        distance = (dl + dr) / 2
        dtheta = (dr - dl) / self.p["wheel_separation"]
        self.x += distance * math.cos(self.theta + dtheta / 2)
        self.y += distance * math.sin(self.theta + dtheta / 2)
        self.theta = math.atan2(
            math.sin(self.theta + dtheta),
            math.cos(self.theta + dtheta),
        )

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion_yaw(self.theta)
        odom.twist.twist.linear.x = (vl + vr) / 2
        odom.twist.twist.angular.z = (
            vr - vl
        ) / self.p["wheel_separation"]
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def _publish_state(self, vl, vr):
        stamp = self.get_clock().now().to_msg()
        radius = self.p["wheel_radius"]

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_state.position = [
            self.left_ticks / self.p["ticks_per_meter"] / radius,
            self.right_ticks / self.p["ticks_per_meter"] / radius,
        ]
        joint_state.velocity = [vl / radius, vr / radius]
        self.joint_pub.publish(joint_state)

        self.left_pub.publish(Int64(data=int(self.left_ticks)))
        self.right_pub.publish(Int64(data=int(self.right_ticks)))

    def destroy_node(self):
        if hasattr(self, "hw"):
            self.hw.stop()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BaseController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
