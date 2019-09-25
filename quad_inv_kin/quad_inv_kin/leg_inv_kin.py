from math import atan2, acos, sqrt, pi
import json
import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from custom.srv import LegInvKin


class LegInvKinServer(Node):
    def __init__(self):
        super().__init__('leg_inv_kin_server')

        self.leg_desc = self.get_leg_desc()

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

    def calc(self, x, y, z):
        alpha = atan2(y, x) # TODO use d
        w = sqrt(x**2 + y**2) - self.leg_desc[1]['a']
        z = z - self.leg_desc[1]['d']
        a = self.leg_desc[2]['a']
        b = self.leg_desc[3]['a']
        r = sqrt(w**2 + z**2)
        delta = acos((a**2 + b**2 - r**2) / (2 * a * b))
        gamma = pi - delta

        b1 = atan2(w, -z)
        b2 = acos((a**2 + r**2 - b**2) / (2 * a * r))
        beta = b1 + b2

        return float(alpha), float(beta - pi/2), float(-gamma)

    def get_leg_desc(self):
        params_path = os.path.join(
            get_package_share_directory('quad_params'),
            'quad_params.json'
        )
        with open(params_path, 'r') as file:
            legs_params = json.load(file)
        return legs_params['general']


def main(args=None):
    rclpy.init(args=args)

    node = LegInvKinServer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
