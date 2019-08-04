import sys
import termios
import tty
import json
import os
from math import atan2, acos, sqrt, pi

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from custom.srv import LegInvKin


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class JoySteer(Node):
    def __init__(self):
        super().__init__('joy_steer')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10
        )
        self.x = 0.05
        self.y = 0.05
        self.z = 0.07

        self.inv_kin_node = rclpy.create_node('leg_inv_kin_client')

    def listener_callback(self, msg):
        print(msg)
        self.x = 0.0 + msg.axes[3] * 0.09
        self.y = 0.0 + msg.axes[2] * 0.09
        self.z = 0.07 + -msg.axes[1] * 0.03
        self.send()

    def send(self):
        cli = self.inv_kin_node.create_client(LegInvKin, 'leg_inv_kin')
        while not cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')

        req = LegInvKin.Request()
        req.x = self.x
        req.y = self.y
        req.z = self.z
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.inv_kin_node, future)

        if future.result() is not None and future.result().success:
            a, b, g = future.result().angles
            legs = ['front_left', 'front_right', 'rear_left', 'rear_right']

            names = []
            positions = []
            for leg in legs:
                names += [
                    f'{leg}_base_to_{leg}_link1',
                    f'{leg}_link1_to_{leg}_link2',
                    f'{leg}_link2_to_{leg}_link3'
                ]
                positions += [a, b, g]

            msg = JointState()
            msg.name = names
            msg.position = positions

            self.get_logger().info('Publishing: ' + str(msg.position))
            self.pub.publish(msg)
        else:
            self.inv_kin_node.get_logger().error(
                'Exception while calling service: %r' % future.exception()
            )


def main(args=None):
    rclpy.init(args=args)

    node = JoySteer()

    rclpy.spin(node)

    node.inv_kin_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
