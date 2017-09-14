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
import time
import tf
from geometry_msgs.msg import TransformStamped

def timerCB(event):
    try:
        (trans,quat) = listener.lookupTransform(map_frame ,base_frame , rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    msg_out = TransformStamped()
    msg_out.header.frame_id = map_frame 
    if sim_mode:
        msg_out.header.stamp = rospy.Time.from_sec(time.time())
    else:
        msg_out.header.stamp = rospy.Time.now()
    msg_out.child_frame_id = robot_id
    msg_out.transform.translation.x = trans[0]
    msg_out.transform.translation.y = trans[1]
    msg_out.transform.translation.z = trans[2]
    msg_out.transform.rotation.x = quat[0]
    msg_out.transform.rotation.y = quat[1]
    msg_out.transform.rotation.z = quat[2]
    msg_out.transform.rotation.w = quat[3]
    pub.publish(msg_out)

if __name__ == '__main__':
    try:
        # ROS Init    
        rospy.init_node('pose_to_transform', anonymous=True)
        
        # Get params
        robot_id = rospy.get_param('~self_id', 'robot_1') # hostname (should be unique in the network)
        sim_mode = rospy.get_param('~sim_mode', 'false') # for simulation
        map_frame = rospy.get_param('~map_frame', 'map') # tf link for map
        base_frame = rospy.get_param('~base_frame', 'base_link') # tf link for robot

        # Pub & Timer
        pub = rospy.Publisher('swarm_poses', TransformStamped, queue_size=10) 
        timer = rospy.Timer(rospy.Duration(0.1), timerCB) # 10Hz   
        
        # TF Listener        
        listener = tf.TransformListener()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Converting pose to transform ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
