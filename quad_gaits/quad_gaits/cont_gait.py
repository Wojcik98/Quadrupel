from math import sin, modf, pi
from time import sleep

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from custom.srv import LegInvKin


def fract(x):
    return modf(x)[0]


class ContGait(Node):
    def __init__(self):
        super().__init__('naive_crawl')
        self.stride = 0.1      # m
        self.beta = 0.9         # duty factor
        self.R = self.stride * self.beta

        tmp = self.beta - 0.5
        self.phi = [
            0,
            0.5,
            self.beta,
            fract(tmp) if tmp >= 0 else 1 - fract(-tmp)
        ]
        self.leg_mapping = {
            0: 'front_left',
            1: 'front_right',
            2: 'rear_left',
            3: 'rear_right',
        }
        self.period = 10.0
        self.t = 0.0

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.inv_kin_cli = self.create_client(LegInvKin, 'leg_inv_kin')
        while not self.inv_kin_cli.wait_for_service(timeout_sec=1.0):
            print('leg_inv_kin service not available, waiting again...')

        self.broadcaster = self.create_publisher(TFMessage, '/tf', 10)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'base_link'
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = 0.1
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 0.0
        self.transform.transform.rotation.w = 1.0

        self.timer_period = 0.1
        while True:
            self.timer_callback()
            sleep(self.timer_period)

    def timer_callback(self):
        phase = fract(self.t / self.period)

        y = 0.03    # relative to leg
        base_z = -0.13

        names = []
        positions = []
        for i in range(4):
            leg = self.leg_mapping[i]
            _, side = leg.split('_')
            y_sign = 1. if side == 'left' else -1.

            if fract(self.phi[i] + phase) < self.beta:  # supporting
                support_phase = fract(self.phi[i] + phase) / self.beta
                x = (self.R / 2.) - self.R * support_phase
                z = base_z    # relative to leg

            else:   # transfer
                transfer_phase = \
                    (fract(self.phi[i] + phase) - self.beta) / (1 - self.beta)
                x = (-self.R / 2.) + self.R * transfer_phase
                z = base_z + 0.05 * sin(transfer_phase * pi)

            req = LegInvKin.Request()
            req.x = x
            req.y = y * y_sign
            req.z = z
            future = self.inv_kin_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            if res is None or not res.success:
                print('leg_inv_kin error')
                continue

            a, b, g = res.angles
            names += [
                f'{leg}_base_to_{leg}_link1',
                f'{leg}_link1_to_{leg}_link2',
                f'{leg}_link2_to_{leg}_link3'
            ]
            positions += [a, b, g]

        msg = JointState()
        msg.name = names
        msg.position = positions

        self.pub.publish(msg)

        self.transform.transform.translation.x += \
            (self.stride / self.period) * self.timer_period
        msg = TFMessage(transforms=[self.transform])
        self.broadcaster.publish(msg)

        self.t += self.timer_period

    def destroy(self):
        self.inv_kin_node.destroy()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    node = ContGait()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
