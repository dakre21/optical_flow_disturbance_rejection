import rclpy
from rclpy.node import Node
from optical_flow_msgs.msg import RpiStatus
from .vcgen import run_command


class RpiStatusNode(Node):

    def __init__(self):
        super().__init__("rpi_status_node")
        self.status_pub = self.create_publisher(RpiStatus, "/rpi_status", 10)

        self.status_timer = self.create_timer(1.0, self.pub_status)

    def pub_status(self):
        msg = RpiStatus()
        msg.throttled = run_command("vcgencmd get_throttled")
        msg.temperature = run_command("vcgencmd measure_temp")
        msg.core_voltage = run_command("vcgencmd measure_volts")
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = RpiStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
