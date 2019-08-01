import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi


class JointStatePub(Node):
    def __init__(self):
        super().__init__('joint_state_pub')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.timer_period = 0.2
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        names = []
        positions = []
        for prefix in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            names += [
                f'{prefix}_base_to_{prefix}_link1',
                f'{prefix}_link1_to_{prefix}_link2',
                f'{prefix}_link2_to_{prefix}_link3'
            ]
            positions += [0.0, -pi / 4, -pi / 4]

        msg = JointState()
        msg.name = names
        msg.position = positions

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_pub = JointStatePub()

    print('Ready!')
    rclpy.spin(joint_state_pub)

    joint_state_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
