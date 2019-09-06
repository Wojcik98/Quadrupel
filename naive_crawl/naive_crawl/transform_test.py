import sys
import termios
import tty
from math import cos, sin

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from custom.srv import Transform


class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_test')

        # self.transform_node = rclpy.create_node('transform_test')
        self.timer_callback()

    def timer_callback(self):
        cli = self.create_client(Transform, 'transform_srv')
        while not cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')

        req = Transform.Request()
        req.source_frame = 'base_link'
        req.target_frame = 'front_left_link_4'
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            print('rcvd resp')
            print(future.result())
        else:
            print('error?')

    def destroy(self):
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    node = TransformNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
