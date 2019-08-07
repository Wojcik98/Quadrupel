import serial
import sys

servo = sys.argv[1]
duty = sys.argv[2]
cmd = f'#{servo}P{duty}T1000\r\n'
raw = cmd.encode('ascii')

port = serial.Serial('/dev/ttyS0')
port.baudrate = 9600

port.write(raw)
port.write(raw)
