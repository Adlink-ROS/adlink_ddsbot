from motor_control import Motor
import time


if __name__ == '__main__':
    m = Motor('/dev/cu.usbmodem1411')
    # spin command format:
    # msg = {'right': {'dir': m.WHEEL_FORWARD, 'val': 50}, 'left': {'dir': m.WHEEL_REVERSE, 'val': 90}}
    for i in range(0, 256, 10):
        msg = dict(right = {'dir': m.WHEEL_FORWARD, 'val': i}, left = {'dir': m.WHEEL_FORWARD, 'val': i})
        m.spin(msg)
        left_value_list = list()
        right_value_list = list()
        for _ in range(10):
            speed_left, speed_right = m.speed()
            #print("raw = {}, left = {}, right = {}, count = {}".format(i, speed_left, speed_right, _))
            left_value_list.append(int(speed_left))
            right_value_list.append(int(speed_right))
            
        left_mean = sum(left_value_list) / len(left_value_list)
        right_mean = sum(right_value_list) / len(right_value_list)
        print("ADC value = {}, mean value: left = {}, right = {}".format(i, left_mean * 18, right_mean * 18))

    msg = dict(right = {'dir': m.WHEEL_FORWARD, 'val': 0}, left = {'dir': m.WHEEL_FORWARD, 'val': 0})
    m.spin(msg)
