import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class StaticSteer(Node):
    def __init__(self):
        super().__init__('static_steer')

        self.broadcaster = self.create_publisher(TFMessage, '/tf', 10)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'base_link'
        self.transform.transform.translation.x = 0.05
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = 0.0
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 0.0
        self.transform.transform.rotation.w = 1.0

        self.timer_period = 0.1
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TFMessage(transforms=[self.transform])
        self.broadcaster.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = StaticSteer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
