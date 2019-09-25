import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage


class JoySteer(Node):
    def __init__(self):
        super().__init__('static_steer')

        self.broadcaster = self.create_publisher(TFMessage, '/tf', 10)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'base_link'
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = 0.0
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 0.0
        self.transform.transform.rotation.w = 1.0

        self.sub = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10
        )
        msg = TFMessage(transforms=[self.transform])
        self.broadcaster.publish(msg)
        print('Ready!')

    def listener_callback(self, msg):
        up = msg.buttons[5] - msg.buttons[7]
        self.transform.transform.translation.x = msg.axes[3] * 0.02
        self.transform.transform.translation.y = msg.axes[2] * 0.03
        self.transform.transform.translation.z = 0.1 + up * 0.03
        self.transform.transform.rotation.x = -msg.axes[0] * 0.1
        self.transform.transform.rotation.y = msg.axes[1] * 0.1
        msg = TFMessage(transforms=[self.transform])
        self.broadcaster.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JoySteer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
