import csv
import serial
import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from custom.msg import ServoCmd, ServoCmdArray


class ServoListener(Node):
    def __init__(self):
        super().__init__('servo_listener')
        self.port = serial.Serial('/dev/ttyS0')
        self.port.baudrate = 9600

        self.centers = [0] * 13
        self.dirs = [1] * 13
        self.load_config()

        self.subscription = self.create_subscription(
            ServoCmdArray,
            'servos',
            self.listener_callback,
            10
        )

        self.cache = []
        self.timer_period = 0.2
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        data = self.cache
        cmd = ''
        i = 0
        time = int(self.timer_period * 1000)
        while i < len(data):
            servo = data[i].number
            angle = data[i].angle
            duty = self.angle_to_duty(servo, angle)

            cmd += f'#{servo}P{duty}T{time}'  # TODO time?

            i += 1

        cmd += '\r\n'
        byts = cmd.encode('ascii')
        self.port.write(byts)

        self.get_logger().info(f'Command sent: "{cmd}"')

    def listener_callback(self, msg):
        self.cache = msg.data

    def angle_to_duty(self, servo, angle):
        duty = self.centers[servo] + int(angle * 1000. / 90.) * self.dirs[servo]
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
                self.centers[i] = int(row['center'])
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
