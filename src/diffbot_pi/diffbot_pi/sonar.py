"""Ultrasonic range sensor node for the DiffBot."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

DEFAULT_PARAMETERS = {
    # Ultrasonic sensor GPIO pins (trigger and echo)
    "trigger_pin": 22,
    "echo_pin": 26,

    # Sensor operating limits
    "max_distance": 3.0,
    "min_range": 0.02,
    "field_of_view": 0.5,

    # Node enablement and ROS output settings
    "enabled": True,
    "topic": "sonar_front",
    "frame_id": "ultrasonic_front_link",
    "publish_hz": 10.0,
}


class Sonar(Node):
    def __init__(self):
        super().__init__("sonar")

        self._declare_parameters()
        self._initialize_sensor()
        self._create_ros_interfaces()

    def _declare_parameters(self):
        for key, value in DEFAULT_PARAMETERS.items():
            self.declare_parameter(key, value)

        self.p = {
            key: self.get_parameter(key).value
            for key in DEFAULT_PARAMETERS
        }

    def _initialize_sensor(self):
        self.sensor = None

        if not self.p["enabled"]:
            self.get_logger().info("Sonar disabled via parameter.")
            return

        try:
            from gpiozero import DistanceSensor
        except ImportError as exc:
            self.get_logger().error(
                "gpiozero is required for the sonar sensor: %s",
                exc,
            )
            return

        self.sensor = DistanceSensor(
            echo=self.p["echo_pin"],
            trigger=self.p["trigger_pin"],
            max_distance=self.p["max_distance"],
        )

    def _create_ros_interfaces(self):
        self.pub = self.create_publisher(Range, self.p["topic"], 10)
        self.create_timer(1.0 / self.p["publish_hz"], self.tick)

    def tick(self):
        if self.sensor is None:
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p["frame_id"]
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self.p["field_of_view"])
        msg.min_range = float(self.p["min_range"])
        msg.max_range = float(self.p["max_distance"])

        try:
            distance = float(self.sensor.distance)
        except Exception as exc:
            self.get_logger().warning("Sonar read failed: %s", exc)
            return

        msg.range = max(msg.min_range, min(distance, msg.max_range))
        self.pub.publish(msg)

    def destroy_node(self):
        if self.sensor is not None:
            self.sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sonar()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
