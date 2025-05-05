import rclpy
import numpy as np
from scipy.signal import chirp, square
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
        self.declare_parameter("type", "chirp")
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
        self.signal = []

        self._create_signal(1)

        self.signal_timer = self.create_timer(self.dt, self.pub_signal)

    def _create_signal(self, sign, n_freq=4, hold_s=5):
        n_samples = int(self.T / self.dt)
        if "chirp" == self.type:
            t = np.linspace(0, self.T, n_samples)
            self.signal = (
                sign
                * self.amp
                * chirp(t, f0=self.f0, f1=self.f1, t1=self.T, method="linear")
            )
        elif "chirp_hold" == self.type:
            n_hold_samples = int(hold_s / self.dt)
            n_chirp_samples = int(n_samples / n_freq) - n_hold_samples
            assert n_chirp_samples > 0

            T_chirp = self.T / n_freq - hold_s
            t_chirp = np.linspace(0, T_chirp, n_chirp_samples)
            t_hold = np.linspace(0, hold_s, n_hold_samples)
            freq = np.linspace(self.f0, self.f1, n_freq)
            for i in range(n_freq - 1):
                chirp_signal = (
                    sign
                    * self.amp
                    * chirp(
                        t_chirp, f0=freq[i], f1=freq[i + 1], t1=T_chirp, method="linear"
                    )
                )
                self.signal.extend(chirp_signal)
                hold_signal = np.ones_like(t_hold) * chirp_signal[-1]
                self.signal.extend(hold_signal)
        elif "square" == self.type:
            t = np.linspace(0, self.T / n_freq, int(n_samples / n_freq))
            freq = np.linspace(self.f0, self.f1, n_freq)
            self.signal = [self.amp * square(2 * np.pi * f * t) for f in freq]
        else:
            self.get_logger().error("Unknown type set")
            rclpy.shutdown()

        self.idx = 0

    def pub_signal(self):
        if self.idx >= len(self.signal):
            if self.symmetric:
                self.symmetric = False
                self._create_signal(-1)
                return

            self.signal_timer.cancel()
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
