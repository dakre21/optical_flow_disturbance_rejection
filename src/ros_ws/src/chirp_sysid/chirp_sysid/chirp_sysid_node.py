import rclpy
import numpy as np
from scipy.signal import chirp
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from mavros_msgs.msg import AttitudeTarget


class ChipSysIdNode(Node):

    def __init__(self):
        super().__init__("chirp_sysid_node")
        self.twist_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )

        self.attitude_pub = self.create_publisher(
            AttitudeTarget, "/mavros/setpoint_raw/attitude", 10
        )

        self.accel_pub = self.create_publisher(
            Vector3Stamped, "/mavros/setpoint_accel/accel", 10
        )

        self.declare_parameter("mode", "v")
        self.declare_parameter("type", "linear")
        self.declare_parameter("dt", 0.01)
        self.declare_parameter("T", 60)
        self.declare_parameter("f0", 0.01)
        self.declare_parameter("f1", 2.0)
        self.declare_parameter("amp", 5.0)
        self.declare_parameter("symmetric", False)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.type = self.get_parameter("type").get_parameter_value().string_value
        self.symmetric = (
            self.get_parameter("symmetric").get_parameter_value().bool_value
        )
        self.T = self.get_parameter("T").value
        self.dt = self.get_parameter("dt").value
        self.f0 = self.get_parameter("f0").value
        self.f1 = self.get_parameter("f1").value
        self.amp = self.get_parameter("amp").value
        self.t0 = self.get_clock().now().nanoseconds / 1e9

        self._create_signal(1)

        self.chirp_timer = self.create_timer(self.dt, self.pub_chirp)

    def _create_signal(self, sign):
        n_samples = int(self.T / self.dt)
        t = np.linspace(0, self.T, n_samples)
        self.signal = (
            sign
            * self.amp
            * chirp(t, f0=self.f0, f1=self.f1, t1=self.T, method=self.type)
        )
        self.idx = 0

    def pub_chirp(self):
        if self.idx >= len(self.signal):
            if self.symmetric:
                self.symmetric = False
                self._create_signal(-1)
                return

            self.chirp_timer.cancel()
            rclpy.shutdown()
            return

        if "v" == self.mode:
            twist = TwistStamped()
            twist.twist.linear.y = self.signal[self.idx]
            self.twist_pub.publish(twist)
        elif "p" == self.mode:
            rad = self.signal[self.idx] * (np.pi / 180)
            rpy = np.array([rad, 0, 0])
            quat = R.from_euler("xyz", rpy).as_quat()
            attitude = AttitudeTarget()
            # attitude.type_mask = AttitudeTarget.IGNORE_THRUST
            attitude.orientation.x = quat[0]
            attitude.orientation.y = quat[1]
            attitude.orientation.z = quat[2]
            attitude.orientation.w = quat[3]
            attitude.thrust = 0.5
            self.attitude_pub.publish(attitude)
        elif "ay" == self.mode:
            accel = Vector3Stamped()
            accel.vector.y = self.signal[self.idx]
            self.accel_pub.publish(accel)
        else:
            self.get_logger().error("Unknown mode set", throttle_duration_sec=1)
            return

        self.idx += 1


def main():
    rclpy.init()
    node = ChipSysIdNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error {e}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
