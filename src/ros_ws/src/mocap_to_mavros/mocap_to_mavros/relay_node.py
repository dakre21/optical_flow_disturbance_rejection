import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies


class RelayNode(Node):

    def __init__(self):
        super().__init__("relay_node")
        self.to_mavros_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", 10
        )
        self.from_mocap_sub = self.create_subscription(
            RigidBodies, "/rigid_bodies", self.rigid_bodies_callback, 10
        )

        self.run_sim = self.from_mocap_sub.get_publisher_count() == 0

        self.rigid_bodies_msg = None

        # (dakre) if we want to run mocap as fast as possible this isnt
        # necessary, but if we want to run it at a slower rate than inner
        # loop for optical flow then bump this down in freq
        self.pub_timer = self.create_timer(1.0, self.pub_pose)

    def pub_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        if self.run_sim:
            msg.pose.position.x = 1.0
            msg.pose.position.y = 1.0
            msg.pose.position.z = -5.0
        elif not self.rigid_bodies_msg:
            self.get_logger().error("No rigid bodies message recv")
        else:
            msg.pose.position.x = msg.rigidbodies[0].pose.position.x
            msg.pose.position.y = msg.rigidbodies[0].pose.position.y
            msg.pose.position.z = msg.rigidbodies[0].pose.position.z

            self.rigid_bodies_msg = None

        self.to_mavros_pub.publish(msg)

    def rigid_bodies_callback(self, msg):
        self.rigid_bodies_msg = msg


def main():
    rclpy.init()
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
