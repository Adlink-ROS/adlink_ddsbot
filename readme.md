# ADLink DDSBot (ROS 1.0/2.0 based swarm robots architecture)   

## Abstract  
Swarm robots using Opensplice DDS with ROS 1&2  
  
## Developer & Team
HaoChih, LIN (haochih.lin@adlinktech.com)  
Chester, Tseng (chester.tseng@adlinktech.com)
Ryan, Chen 
  
ADLINK Technology, Inc  
Advanced Robotic Platform Group  

## License
Apache 2.0  
Copyright 2017 ADLINK Technology, Inc.  

## Tutorial
### Simulation (stage)
Environment requirements:  
* ROS 1.0 (Kinetic)  
* ROS 2.0 (tested on r2b2)  
* catkin_ws  
  
1. Ckeck the hostname (should be unique)  
2. open a terminal (remember to source ros2 & ros1 setup.bash)  
3. $ export HOSTNAME
4. $ roslaunch adlink_ddsbot Swarm_Sim_Stage.launch  
5. $ ros2 run ros1_bridge parameter_bridge  
6. Repeat the same steps on another computer/robot   
(both should be in the same network)  
7. Then, each robot can be aware of all the others shown on the rviz.





