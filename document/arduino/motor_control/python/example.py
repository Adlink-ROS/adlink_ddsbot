from motor_control import Motor
import time


if __name__ == '__main__':
    m = Motor('/dev/cu.usbmodem1411')
    # spin command format:
    # msg = {'right': {'dir': m.WHEEL_FORWARD, 'val': 50}, 'left': {'dir': m.WHEEL_REVERSE, 'val': 90}}
    for i in [300, 500, 700, 900, 1100, 1300, 1500, 1700, 0]:
        msg = dict(right = {'dir': m.WHEEL_FORWARD, 'val': i}, left = {'dir': m.WHEEL_FORWARD, 'val': i})
        m.spin(msg)
        time.sleep(3)

    for i in [300, 500, 700, 900, 1100, 1300, 1500, 1700, 0]:
        msg = dict(right = {'dir': m.WHEEL_REVERSE, 'val': i}, left = {'dir': m.WHEEL_REVERSE, 'val': i})
        m.spin(msg)
        time.sleep(3)

    try:
        print(m.speed())
    except:
        print("get motor speed failed")
