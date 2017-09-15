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
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

def poseCB(msg_in):
    msg_out = TransformStamped()
    msg_out.header = msg_in.header 
    msg_out.child_frame_id = robot_id
    msg_out.transform.translation.x = msg_in.pose.pose.position.x
    msg_out.transform.translation.y = msg_in.pose.pose.position.y
    msg_out.transform.translation.z = msg_in.pose.pose.position.z
    msg_out.transform.rotation = msg_in.pose.pose.orientation   
    if sim_mode:
        msg_out.header.stamp = rospy.Time.from_sec(time.time())
    pub.publish(msg_out)

if __name__ == '__main__':
    try:
        # ROS Init    
        rospy.init_node('pose_to_transform', anonymous=True)
        
        # Get params
        robot_id = rospy.get_param('~self_id', 'robot_1') # hostname
        sim_mode = rospy.get_param('~sim_mode', 'false') # for simulation

        # Pub & Sub
        pub = rospy.Publisher('swarm_poses', TransformStamped, queue_size=10)    
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, poseCB)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Converting pose to transform ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
