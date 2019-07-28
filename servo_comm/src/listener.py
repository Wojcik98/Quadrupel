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

    def listener_callback(self, msg):
        cmd = ''
        i = 0
        while i < len(msg.data):
            servo = msg.data[i].number
            angle = msg.data[i].angle
            time = msg.data[i].time
            duty = self.angle_to_duty(servo, angle)

            cmd += f'#{servo}P{duty}T{time}'  # TODO time?

            i += 1

        cmd += '\r\n'
        byts = cmd.encode('ascii')
        self.port.write('\r\n'.encode('ascii'))
        self.port.write(byts)

        self.get_logger().info(f'Command sent: "{cmd}"')

    def angle_to_duty(self, servo, angle):
        if angle < -90.:    # is it needed?
            angle = -90.
        elif angle > 90.:
            angle = 90.

        duty = self.centers[servo] + int(angle * 1000. / 90.) * self.dirs[servo]
        return duty


def main(args=None):
    rclpy.init(args=args)

    servo_listener = ServoListener()

    rclpy.spin(servo_listener)

    servo_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
