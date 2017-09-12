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
from adlink_ddsbot.msg import MultiRobots, Robot
from geometry_msgs.msg import TransformStamped

class RobotIdFilter:
    def __init__(self, selfId, mapId, id_timeout, sim_mode, fake_reliability, fake_radius):
        self.sub = rospy.Subscriber('swarm_poses', TransformStamped, self.poseCB, queue_size=10)
        self.pub = rospy.Publisher('multi_robots', MultiRobots, queue_size=10)   
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCB) # 10Hz
        self.selfId = selfId
        self.mapId = mapId
        self.id_timeout = id_timeout
        self.sim_mode = sim_mode
        self.robotId_Dict = {}
        self.fake_reliability = fake_reliability
        self.fake_radius = fake_radius

    def poseCB(self, data):
        if data.child_frame_id == self.selfId:
            return
        # Update robot id in dict
        self.robotId_Dict[data.child_frame_id] = data
    
    def timerCB(self, event):
        if not self.robotId_Dict: # check if dict is empty or not 
            return
        # Check the timeout of each robot id, then publish the data
        curr_time = rospy.get_time()
        if sim_mode:
            curr_time = (rospy.Time.from_sec(time.time())).to_sec()
        msg = MultiRobots()
        msg.header.stamp = rospy.Time.now()
        for key, value in self.robotId_Dict.items():
            if (curr_time - value.header.stamp.to_sec()) >= self.id_timeout:
                del self.robotId_Dict[key]
                print value.child_frame_id + ' is too old, removed!' # for debug
            else:
                robot = Robot()
                robot.transform = value
                robot.reliability = self.fake_reliability
                robot.radius = self.fake_radius
                msg.robots.append(robot)
        self.pub.publish(msg)

if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('robot_id_filter', anonymous=True)

        # Get params
        self_id = rospy.get_param('~self_id', 'robot_1') # robot frame id
        map_id = rospy.get_param('~map_id', 'map') # map frame id (default is map)
        id_timeout = float( rospy.get_param('~id_timeout', '2') ) # unit: sec
        sim_mode = rospy.get_param('~sim_mode', 'false') # for simulation
        fake_reliability = float( rospy.get_param('~fake_reliability', '0.9') ) # 0 ~ 1 (<=0 disable)
        fake_radius = float( rospy.get_param('~fake_radius', '0.12') ) # unit: meter (<=0 disable)
        if id_timeout <= 0:
            id_timeout = 0
        if fake_reliability <= 0:
            fake_reliability = 0.9
        if fake_radius <= 0:
            fake_radius = 0.12

        # Constract RobotIdFilter Obj
        rif = RobotIdFilter(self_id, map_id, id_timeout, sim_mode, fake_reliability, fake_radius)
        rospy.loginfo("Start Robot Id Filter ...")
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
