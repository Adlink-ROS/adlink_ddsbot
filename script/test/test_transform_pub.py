#!/usr/bin/python
# Copyright 2017 ADLINK Technology, Inc.
# Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
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

import rospy
from geometry_msgs.msg import TransformStamped

def transformPub():
    pub = rospy.Publisher('swarm_poses', TransformStamped, queue_size=10)
    rospy.init_node('test_transform_pub', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    count_sw = True
    while not rospy.is_shutdown():
        if count_sw is True:
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'robot_2'
            msg.transform.translation.x = 47.3
            msg.transform.translation.y = 18.3
            msg.transform.translation.z = 0.0
            pub.publish(msg)
            rate.sleep()
            count_sw = False
        else: 
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'robot_3'
            msg.transform.translation.x = 40.3
            msg.transform.translation.y = 18.3
            msg.transform.translation.z = 0.0
            pub.publish(msg)
            rate.sleep()
            count_sw = True

if __name__ == '__main__':
    try:
        transformPub()
    except rospy.ROSInterruptException:
        pass
