import csv
import serial
import os
from math import pi

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState


class ServoListener(Node):
    joint_to_servo = {
        f'rear_right_base_to_rear_right_link1': 10,
        f'rear_right_link1_to_rear_right_link2': 11,
        f'rear_right_link2_to_rear_right_link3': 12,
        f'rear_left_base_to_rear_left_link1': 7,
        f'rear_left_link1_to_rear_left_link2': 8,
        f'rear_left_link2_to_rear_left_link3': 9,
        f'front_right_base_to_front_right_link1': 4,
        f'front_right_link1_to_front_right_link2': 5,
        f'front_right_link2_to_front_right_link3': 6,
        f'front_left_base_to_front_left_link1': 1,
        f'front_left_link1_to_front_left_link2': 2,
        f'front_left_link2_to_front_left_link3': 3,
    }

    def __init__(self):
        super().__init__('servo_listener')
        self.port = serial.Serial('/dev/ttyS0')
        self.port.baudrate = 9600

        self.centers = [0] * 13
        self.dirs = [1] * 13
        self.load_config()

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )

        self.cache = None
        self.prev = None
        self.timer_period = 0.2
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.cache is None or self.cache == self.prev:
            return

        data = self.cache
        cmd = ''
        i = 0
        time = int(self.timer_period * 1000)
        while i < len(data.name):
            joint = data.name[i]
            servo = self.joint_to_servo[joint]
            angle = data.position[i]
            duty = self.angle_to_duty(servo, angle)

            cmd += f'#{servo}P{duty}T{time}'

            i += 1

        cmd += '\r\n'
        byts = cmd.encode('ascii')
        self.port.write(byts)
        self.get_logger().info(f'Command sent: "{cmd}"')

        self.prev = self.cache

    def listener_callback(self, msg):
        self.cache = msg

    def angle_to_duty(self, servo, angle):
        displacement = int(angle * 1000. / (pi / 2.)) * self.dirs[servo]
        duty = self.centers[servo] + displacement
        if duty < 0 and duty + 4000 <= 2550:
            duty += 4000    # reverse direction
        duty = max(duty, 460)
        duty = min(duty, 2550)
        return duty

    def load_config(self):
        path = os.path.join(
            get_package_share_directory('servo_comm'),
            'servo_center.csv'
        )

        with open(path) as file:
            reader = csv.DictReader(file)
            i = 1
            for row in reader:
                self.centers[i] = int(row['zero'])
                self.dirs[i] = int(row['dir'])
                i += 1


def main(args=None):
    rclpy.init(args=args)

    servo_listener = ServoListener()

    print('Ready!')
    rclpy.spin(servo_listener)

    servo_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
