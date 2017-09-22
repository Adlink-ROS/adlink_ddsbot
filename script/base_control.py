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
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class BaseControl:
    def __init__(self, baseId, odomId, device_port, baudrate, wheel_separation, wheel_radius, vx_cov, vyaw_cov, debug_mode):
        # Serial Communication
        try:
            self.serial = serial.Serial(device_port, baudrate, timeout=10)
            rospy.loginfo("Flusing first 50 data reading ...")
            for x in range(0, 50):
                line = self.serial.readline()
                data_array = string.split(line,",")    # Fields split
                time.sleep(0.01)
        except serial.serialutil.SerialException:
            rospy.logerr("Can not receive data from the port: "+port + 
            ". Did you specify the correct port in the launch file?")
            self.serial.close
            sys.exit(0) 
        rospy.loginfo("Communication success !")
        # ROS handler        
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmdCB, queue_size=10)
        self.pub = rospy.Publisher('base_odom', Odometry, queue_size=10)   
        self.timer_odom = rospy.Timer(rospy.Duration(0.02), self.timerOdomCB) # 50Hz
        self.timer_cmd = rospy.Timer(rospy.Duration(0.1), self.timerCmdCB) # 10Hz
        # from rosparam        
        self.wheelSep = wheel_separation
        self.wheelRad = wheel_radius
        self.baseId = baseId
        self.odomId = odomId
        self.VxCov = vx_cov
        self.VyawCov = vyaw_cov
        self.debug_mode = debug_mode
        # variable        
        self.trans_x = 0.0
        self.rotat_z = 0.0
        self.WL_send = 0.0
        self.WR_send = 0.0 

    def cmdCB(self, data):
        self.trans_x = data.linear.x
        self.rotat_z = data.angular.z
    
    def timerOdomCB(self, event):
        # Serial read & publish 
        try:
            line = self.serial.readline()
            data_array = string.split(line,",") # Fields split
            # Normal mode            
            if self.debug_mode == False: 
                if len(data_array) == 2:
                    WL = float(data_array[0]) # unit: deg/sec
                    WR = float(data_array[1])
                #print str(WL/18) + ', ' + str(WR/18) # for debug, unit: pulse/sec            

                VL = WL * math.pi/180.0 * self.wheelRad # V = omega * radius, unit: m/s
                VR = WR * math.pi/180.0 * self.wheelRad

                Vyaw = (VR-VL)/self.wheelSep
                Vx = (VR+VL)/2.0

                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.odomId
                msg.child_frame_id = self.baseId
                msg.twist.twist.linear.x = Vx
                msg.twist.twist.angular.z = Vyaw
                for i in range(36):
                    msg.twist.covariance[i] = 1e6
                msg.twist.covariance[0] = self.VxCov
                msg.twist.covariance[35] = self.VyawCov
                self.pub.publish(msg)
            # Debug mode            
            if self.debug_mode == True: 
                if len(data_array) == 12:
                    L_ref = float(data_array[0]) # deg/sec
                    R_ref = float(data_array[1])
                    L_err = float(data_array[2]) # deg/sec
                    R_err = float(data_array[3])
                    L_int = float(data_array[4]) # deg/sec
                    R_int = float(data_array[5]) 
                    L_ave = float(data_array[6]) # deg/sec
                    R_ave = float(data_array[7]) 
                    pwm_1 = float(data_array[8]) 
                    pwm_2 = float(data_array[9])
                    pwm_3 = float(data_array[10]) 
                    pwm_4 = float(data_array[11])
                rospy.loginfo("[Debug] Lsend:%4d, Rsend:%4d, Lref:%4d, Rref:%4d, Lerr:%4d, Rerr:%4d, Lint:%4d, Rint:%4d, Lave:%3d, Rave:%3d, pwm: %3d %3d %3d %3d",
                              self.WL_send,self.WR_send, L_ref,R_ref, L_err,R_err, L_int,R_int, L_ave,R_ave, pwm_1,pwm_2,pwm_3,pwm_4 )
        except: 
            #rospy.loginfo("Error in sensor value !")       
            pass            

    def timerCmdCB(self, event):
        # send cmd to motor
        VR = self.trans_x + self.wheelSep/2.0*self.rotat_z; # unit: m/s
        VL = self.trans_x - self.wheelSep/2.0*self.rotat_z;        
        self.WR_send = VR/self.wheelRad * 180.0/math.pi # unit: deg/sec
        self.WL_send = VL/self.wheelRad * 180.0/math.pi
        values = [str(self.WL_send), str(self.WR_send), '0']
        values[2] = str(len(values[0]) + len(values[1]))
        cmd = ",".join(values).encode()        
        self.serial.flushInput()
        self.serial.write(cmd)
        
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('base_control', anonymous=True)

        # Get params
        baseId = rospy.get_param('~base_id', 'base_link') # base link
        odomId = rospy.get_param('~odom_id', 'base_odom')      # odom link
        device_port = rospy.get_param('~port', '/dev/uno') # device port
        baudrate = float( rospy.get_param('~baudrate', '115200') ) 
        wheel_separation = float( rospy.get_param('~wheel_separation', '0.15') ) # unit: meter 
        wheel_radius = float( rospy.get_param('~wheel_radius', '0.0335') ) # unit: meter
        vx_cov = float( rospy.get_param('~vx_cov', '0.01') ) # covariance for Vx measurement
        vyaw_cov = float( rospy.get_param('~vyaw_cov', '0.5') ) # covariance for Vyaw measurement
        debug_mode = bool( rospy.get_param('~debug_mode', 'false') ) # true for detail info

        # Constract BaseControl Obj
        rospy.loginfo("Start base control node ...")
        bc = BaseControl(baseId, odomId, device_port, baudrate, wheel_separation, wheel_radius, vx_cov, vyaw_cov, debug_mode)
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("shutting down")
