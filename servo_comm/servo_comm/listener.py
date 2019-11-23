import csv
import serial
import os
from math import pi

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState


class ServoConfig:
    def __init__(self, center, dir_):
        self.pin = 0
        self.center = center
        self.dir_ = dir_

    def conn(self, pin):
        self.pin = pin
        return self


class ServoListener(Node):
    joint_to_servo = {
        f'rear_right_base_to_rear_right_link1': ServoConfig(500, -1).conn(3),
        f'rear_right_link1_to_rear_right_link2': ServoConfig(880, -1).conn(4),
        f'rear_right_link2_to_rear_right_link3': ServoConfig(1050, -1).conn(5),
        f'rear_left_base_to_rear_left_link1': ServoConfig(2500, -1).conn(0),
        f'rear_left_link1_to_rear_left_link2': ServoConfig(2320, 1).conn(1),
        f'rear_left_link2_to_rear_left_link3': ServoConfig(1880, 1).conn(2),
        f'front_right_base_to_front_right_link1': ServoConfig(500, -1).conn(9),
        f'front_right_link1_to_front_right_link2': ServoConfig(2320, 1).conn(10),
        f'front_right_link2_to_front_right_link3': ServoConfig(1900, 1).conn(11),
        f'front_left_base_to_front_left_link1': ServoConfig(2430, -1).conn(6),
        f'front_left_link1_to_front_left_link2': ServoConfig(830, -1).conn(7),
        f'front_left_link2_to_front_left_link3': ServoConfig(1150, -1).conn(8),
    }

    def __init__(self):
        super().__init__('servo_listener')
        self.port = serial.Serial('/dev/ttyS0')
        self.port.baudrate = 115200

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )

        self.cache = None
        self.prev = None
        self.timer_period = 1. / 100
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.cache is None or self.cache == self.prev:
            return

        data = self.cache
        cmd = bytearray()
        i = 0
        while i < len(data.name):
            joint = data.name[i]
            servo_config = self.joint_to_servo[joint]
            angle = data.position[i]
            duty = self.angle_to_duty(servo_config, angle)

            cmd += self.make_cmd(servo_config.pin, duty)   # TODO set multiple targets

            i += 1

        self.port.write(cmd)
        self.get_logger().info(f'Command sent: "{cmd}"')

        self.prev = self.cache

    def make_cmd(self, servo_pin, duty):
        mask = 2**7 - 1 # last 7 bits
        duty *= 4
        low = duty & mask
        high = int(duty / (2**7)) & mask
        return bytearray([0x84, servo_pin, low, high])

    def listener_callback(self, msg):
        self.cache = msg

    def angle_to_duty(self, servo_config, angle):
        displacement = int(angle * 1000. / (pi / 2.)) * servo_config.dir_
        duty = servo_config.center + displacement
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
            for row in reader:
                servo = int(row['servo'])
                self.centers[servo] = int(row['zero'])
                self.dirs[servo] = int(row['dir'])


def main(args=None):
    rclpy.init(args=args)

    servo_listener = ServoListener()

    print('Ready!')
    rclpy.spin(servo_listener)

    servo_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
