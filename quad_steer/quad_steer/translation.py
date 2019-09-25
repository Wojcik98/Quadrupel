from time import sleep

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from custom.srv import Transform, LegInvKin

import numpy as np


def quaternion_to_matrix(q):
    x, y, z, w = q

    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2],
    ])


class TranslationNode(Node):
    def __init__(self):
        super().__init__('translation')
        dx = 0.05
        dy = 0.08
        dz = 0.0
        odom_2_front_left = (dx, dy, dz)
        odom_2_front_right = (dx, -dy, dz)
        odom_2_rear_left = (-dx, dy, dz)
        odom_2_rear_right = (-dx, -dy, dz)

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

        self.broadcaster = self.create_publisher(TFMessage, '/tf_static', 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        while True:
            sleep(0.02)
            self.timer_callback()

    def timer_callback(self):
        self.send_legs()
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
        req.target_frame = leg
        future = self.transform_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            print('transform error')
            return (0.0, 0.0, 0.0)

        x, y, z = (res.x, res.y, res.z)

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

    def send_legs(self):
        transforms = []
        for leg, position in self.odom2.items():
            transform = TransformStamped()
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            transform.header.frame_id = 'odom'
            transform.child_frame_id = leg
            transform.transform.translation.x = position[0]
            transform.transform.translation.y = position[1]
            transform.transform.translation.z = position[2]
            transforms.append(transform)

        msg = TFMessage(transforms=transforms)
        self.broadcaster.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TranslationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
