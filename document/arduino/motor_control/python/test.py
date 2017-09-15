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
