import serial
from serial.tools import list_ports
import json
import logging

def get_ports_info():
    ports = list_ports.comports()
    for port in ports:
        print("{} {}".format(port.device, port.usb_info()))

class Motor:
    WHEEL_LEFT = 0
    WHEEL_RIGHT = 1
    WHEEL_FORWARD = 2
    WHEEL_REVERSE = 3
    def __init__(self, comport_name, baudrate=115200):
        if not isinstance(comport_name, str):
            raise ValueError("Invalid comport name")
        self.serial = serial.Serial(comport_name, baudrate)

    def spin(self, command):
        left_cmd = command.get('left')
        right_cmd = command.get('right')
        values = ["0", "0", "0", "0"]

        if left_cmd is not None:
            direction = left_cmd.get('dir')
            value = left_cmd.get('val')
            if direction == self.WHEEL_FORWARD:
                values[0] = str(value)
            elif direction == self.WHEEL_REVERSE:
                values[1] = str(value)

        if right_cmd is not None:
            direction = right_cmd.get('dir')
            value = right_cmd.get('val')
            if direction == self.WHEEL_FORWARD:
                values[2] = str(value)
            elif direction == self.WHEEL_REVERSE:
                values[3] = str(value)

        msg = ",".join(values).encode()
        self.serial.write(msg)

    def speed(self):
        msg = self.serial.readline().strip().decode('utf-8')
        speed_left, speed_right = msg.split(',')
        return speed_left, speed_right
