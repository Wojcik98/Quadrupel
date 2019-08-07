import sys
import termios
import tty
from math import cos, sin

import rclpy
from rclpy.node import Node

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


class NaiveCrawl(Node):
    def __init__(self):
        super().__init__('naive_crawl')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.t = 0.0
        self.inv_kin_node = rclpy.create_node('leg_inv_kin_client')
        self.timer_period = 0.1
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        cli = self.inv_kin_node.create_client(LegInvKin, 'leg_inv_kin')
        while not cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')

        y = 0.06
        x = 0. + 0.03 * cos(2*self.t)
        z = 0.0 + 0.02 * sin(2*self.t)

        req = LegInvKin.Request()
        req.x = x
        req.y = y
        req.z = z
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.inv_kin_node, future)

        if future.result() is not None and future.result().success:
            a, b, g = future.result().angles
            left_legs = ['front_left', 'rear_left']
            right_legs = ['front_right', 'rear_right']

            names = []
            positions = []
            for leg in left_legs:
                names += [
                    f'{leg}_base_to_{leg}_link1',
                    f'{leg}_link1_to_{leg}_link2',
                    f'{leg}_link2_to_{leg}_link3'
                ]
                positions += [a, b, g]

            for leg in right_legs:
                names += [
                    f'{leg}_base_to_{leg}_link1',
                    f'{leg}_link1_to_{leg}_link2',
                    f'{leg}_link2_to_{leg}_link3'
                ]
                positions += [-a, b, g]

            msg = JointState()
            msg.name = names
            msg.position = positions

            self.get_logger().info('Publishing: ' + str(msg.position))
            self.pub.publish(msg)
        else:
            self.inv_kin_node.get_logger().error(
                'Exception while calling service: %r' % future.exception()
            )
        self.t += self.timer_period

    def destroy(self):
        self.inv_kin_node.destroy()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    node = NaiveCrawl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
