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
from geometry_msgs.msg import PoseStamped, TransformStamped

class GoalFilter:
    def __init__(self, self_id, sim_mode):
        self.sub = rospy.Subscriber('swarm_goals', TransformStamped, self.goalCB, queue_size=10)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   
        self.selfId = self_id
        self.sim_mode = sim_mode

    def goalCB(self, data):
        if data.child_frame_id == self.selfId:
            goal = PoseStamped()
            goal.header = data.header
            if sim_mode:
                goal.header.stamp = rospy.Time.now()
            goal.pose.position = data.transform.translation
            goal.pose.orientation = data.transform.rotation
            self.pub.publish(goal)

if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('goal_filter', anonymous=True)

        # Get params
        self_id = rospy.get_param('~self_id', 'robot_1') # robot frame id
        sim_mode = rospy.get_param('~sim_mode', 'false') # for simulation

        # Constract GoalFilter Obj
        gf = GoalFilter(self_id, sim_mode)
        rospy.loginfo("Start Goal Filter ...")
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
