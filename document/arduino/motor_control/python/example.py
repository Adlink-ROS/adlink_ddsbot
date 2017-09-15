#!/usr/bin/python
# Copyright 2017 ADLINK Technology, Inc.
# Developer: Chester, Tseng (haochih.lin@adlinktech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from motor_control import Motor
import time


if __name__ == '__main__':
    m = Motor('/dev/uno')
    # spin command format:
    # msg = {'right': {'dir': m.WHEEL_FORWARD, 'val': 50}, 'left': {'dir': m.WHEEL_REVERSE, 'val': 90}}
    """    
    for i in [300, 500, 700, 900, 1100, 1300, 1500, 1700, 0]:
        msg = dict(right = {'dir': m.WHEEL_FORWARD, 'val': i}, left = {'dir': m.WHEEL_FORWARD, 'val': i})
        m.spin(msg)
        time.sleep(3)

    for i in [300, 500, 700, 900, 1100, 1300, 1500, 1700, 0]:
        msg = dict(right = {'dir': m.WHEEL_REVERSE, 'val': i}, left = {'dir': m.WHEEL_REVERSE, 'val': i})
        m.spin(msg)
        time.sleep(3)
    """
    for i in range(500):
        try:
            print(m.speed())
        except:
            print("get motor speed failed")
        time.sleep(0.01)
