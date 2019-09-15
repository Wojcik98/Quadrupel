from time import sleep

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from custom.srv import Transform, LegInvKin

import numpy as np


class TranslationNode(Node):
    def __init__(self):
        super().__init__('translation')
        dx = 0.05
        dy = 0.08
        dz = 0.0
        odom_2_front_left = np.eye(4)
        odom_2_front_left[0:3, 3] = (dx, dy, dz)
        odom_2_front_right = np.eye(4)
        odom_2_front_right[0:3, 3] = (dx, -dy, dz)
        odom_2_rear_left = np.eye(4)
        odom_2_rear_left[0:3, 3] = (-dx, dy, dz)
        odom_2_rear_right = np.eye(4)
        odom_2_rear_right[0:3, 3] = (-dx, -dy, dz)

        self.odom2 = {
            'front_left': odom_2_front_left,
            'front_right': odom_2_front_right,
            'rear_left': odom_2_rear_left,
            'rear_right': odom_2_rear_right,
        }

        self.transform_cli = self.create_client(Transform, 'transform_srv')
        while not self.transform_cli.wait_for_service(timeout_sec=1.0):
            print('transform_srv service not available, waiting again...')

        self.inv_kin_cli = self.create_client(LegInvKin, 'leg_inv_kin')
        while not self.inv_kin_cli.wait_for_service(timeout_sec=1.0):
            print('leg_inv_kin service not available, waiting again...')

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        while True:
            sleep(0.02)
            self.timer_callback()

    def timer_callback(self):
        legs = ['front_left', 'front_right', 'rear_left', 'rear_right']

        names = []
        positions = []
        for leg in legs:
            a, b, g = self.leg_inv(leg)
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

    def leg_inv(self, leg):
        req = Transform.Request()
        req.source_frame = f'{leg}_base'
        req.target_frame = 'odom'
        future = self.transform_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            print('transform error')
            return (0.0, 0.0, 0.0)

        leg_base = (res.x, res.y, res.z)
        leg_base_2_odom = np.eye(4)
        leg_base_2_odom[0:3, 3] = leg_base
        leg_base_2_leg = np.matmul(leg_base_2_odom, self.odom2[leg])

        x, y, z = leg_base_2_leg[0:3, 3]\

        req = LegInvKin.Request()
        req.x = x
        req.y = y
        req.z = z
        future = self.inv_kin_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None or not future.result().success:
            print('leg_inv_kin error')
            return (0.0, 0.0, 0.0)

        return future.result().angles

    def destroy(self):
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    node = TranslationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
