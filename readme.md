# ADLink DDSBot (ROS 1.0/2.0 based swarm robots architecture)   

## Abstract  
Swarm robots using Opensplice DDS with ROS 1&2  
  
## Developer & Team
HaoChih, LIN (haochih.lin@adlinktech.com)  
Chester, Tseng (chester.tseng@adlinktech.com)  
Erik, Boasson  
Ryan, Chen   
  
ADLINK Technology, Inc  
Advanced Robotic Platform Group  

## License
Apache 2.0  
Copyright 2017 ADLINK Technology, Inc.  

## Tutorial
### System prerequisite
Desktop:  
* ROS 1.0  
  (Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu)  
* ROS 2.0 (tested on r2b2: )  
* create your own catkin_ws  
  (http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)  
* Install required dependencies  
  $ sudo apt-get install 

### Simulation (stage, two computers)  
1. Ckeck the hostname (should be unique in the network)  
2. Check the system time synchronization ($ sudo ntpdate IP_TARGET)  
3. Open a terminal (remember to source ros2 & ros1 setup.bash)  
4. $ export HOSTNAME
5. $ roslaunch adlink_ddsbot Swarm_Sim_Stage.launch  
6. $ ros2 run ros1_bridge parameter_bridge  
7. Repeat the same steps on another computer/robot   
   (both should be in the same network)  
8. Then, each robot can be aware of all the others shown on the rviz.





