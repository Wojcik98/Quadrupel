from math import atan2, acos, sqrt, pi

import rclpy
from rclpy.node import Node

from custom.srv import LegInvKin


class LegInvKinServer(Node):

    def __init__(self):
        super().__init__('leg_inv_kin_server')
        self.srv = self.create_service(
            LegInvKin, 'leg_inv_kin', self.leg_inv_kin_callback
        )

    def leg_inv_kin_callback(self, cmd, response):
        x = cmd.x
        y = cmd.y
        z = cmd.z

        try:
            a, b, g = self.calc(x, y, z)
        except ValueError:
            response.success = False
            response.angles = [0., 0., 0.]
            return response

        response.success = True
        response.angles = [a, b, g]
        self.get_logger().info(f'Inverse kinematics: {a}, {b}, {g}')

        return response

    def calc(self, x, y, z):   # TODO z, read from dh.json
        alpha = atan2(y, x)
        xy = sqrt(x**2 + y**2)
        a = 0.066
        b = 0.052
        r = sqrt(xy**2 + z**2)
        delta = acos((a**2 + b**2 - r**2) / (2 * a * b))
        gamma = pi - delta

        b1 = atan2(xy, -z)
        b2 = acos((a**2 + r**2 - b**2) / (2 * a * r))
        beta = b1 + b2

        return float(alpha), float(beta - pi/2), float(-gamma)


def main(args=None):
    rclpy.init(args=args)

    node = LegInvKinServer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
