from math import cos, sin, modf, pi
from time import sleep

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from custom.srv import LegInvKin


def fract(x):
    return modf(x)[0]


class CrabGait(Node):
    def __init__(self):
        super().__init__('crab_gait')
        self.stride = 0.10      # m
        self.beta = 0.8         # duty factor
        self.R = self.stride * self.beta
        self.alpha = -pi * 1 / 8

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
        self.period = 5.0
        self.t = 0.0

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.inv_kin_cli = self.create_client(LegInvKin, 'leg_inv_kin')
        while not self.inv_kin_cli.wait_for_service(timeout_sec=1.0):
            print('leg_inv_kin service not available, waiting again...')

        self.base_x = 0.03
        self.base_y = 0.04
        self.base_z = -0.13

        self.broadcaster = self.create_publisher(TFMessage, '/tf', 10)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'base_link'
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = -self.base_z
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
        transfer_time = 1.0 - self.beta

        base_x, base_y, base_z = self.base_x, self.base_y, self.base_z

        ca, sa = cos(self.alpha), sin(self.alpha)

        names = []
        positions = []
        for i in range(4):
            changed = False
            moved = False
            dx = 0.0
            dy = 0.0
            leg = self.leg_mapping[i]
            front, side = leg.split('_')
            x_sign = 1. if front == 'front' else -1.
            y_sign = 1. if side == 'left' else -1.

            leg_phase = phase - self.phi[i]
            if leg_phase < 0.0:
                leg_phase += 1.0

            if leg_phase >= self.beta:  # transfer
                changed = True
                transfer_phase = \
                    (leg_phase - self.beta) / (1 - self.beta)
                dr = (-self.R / 2.) + self.R * transfer_phase
                dx = dr * ca
                dy = dr * sa
                x = x_sign * base_x + dx
                y = y_sign * base_y + dy
                z = base_z + 0.05 * sin(transfer_phase * pi)
            elif (0.0 <= phase and phase <= (0.5 - 2 * transfer_time)):  # move
                changed = True
                moved = True
                move_phase = phase / (0.5 - 2 * transfer_time)
                dr = (-self.R / 2.) * move_phase
                dx = dr * ca
                dy = dr * sa
                x = \
                    x_sign * base_x + \
                    (side == 'left') * (self.R / 2.) * ca + \
                    dx
                y = \
                    y_sign * base_y + \
                    (side == 'left') * (self.R / 2.) * sa + \
                    dy
                z = base_z    # relative to leg
            elif (0.5 <= phase and phase <= (1.0 - 2 * transfer_time)):  # move
                changed = True
                moved = True
                move_phase = (phase - 0.5) / (0.5 - 2 * transfer_time)
                dr = (-self.R / 2.) * move_phase
                dx = dr * ca
                dy = dr * sa
                x = \
                    x_sign * base_x + \
                    (side == 'right') * (self.R / 2.) * ca + \
                    dx
                y = \
                    y_sign * base_y + \
                    (side == 'right') * (self.R / 2.) * sa + \
                    dy
                z = base_z    # relative to leg

            if changed:
                req = LegInvKin.Request()
                req.x = x
                req.y = y
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

        if moved:
            self.transform.transform.translation.x += \
                -dx * self.timer_period
            self.transform.transform.translation.y += \
                -dy * self.timer_period

        msg = JointState()
        msg.name = names
        msg.position = positions

        self.pub.publish(msg)

        msg = TFMessage(transforms=[self.transform])
        self.broadcaster.publish(msg)

        self.t += self.timer_period

    def destroy(self):
        self.inv_kin_node.destroy()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    node = CrabGait()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
