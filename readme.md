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

### Real robot (raspberryPi3)
[Hardware]  
Build your own low-cost ddsbot (~250USD)  
link:   

[Image files]  
Download pre-built image for Pi3 (ubuntu mate 16.04, ros 1/2)  
link:   
Burn image to SD card  

[Setup steps]  
1. Connect Pi3 with your desktop by either ethernet cable or debug cable  
2. Power-on and login: (passward is: adlinkros)   
   * hdmi display: you can see the ADLink logo on the desktop  
   * ethernet: the default id address of provided image is 10.0.0.1  
               ($ ssh pi@10.0.0.1)   
   * debug cable: enter the passward  
3. Change robot's hostname and connect to local network  
4. Synchronize the system time throught AP server/specific computer  
   ($ sudo ntpdate IP_ADDRESS)   
4. Change the ROS 1 multi-machines variables in .bashrc  
   (for each robot, it has own roscore)  
5. For ROS 1 Nav: $ roslaunch adlink_ddsbot Adlink_DDSBot_Nav_Single.launch  
6. For ROS 2 Swarm:   
   * $ roslaunch adlink_ddsbot Adlink_DDSBot_Nav_Swarm.launch  
   * $ ros2 run ros1_bridge parameter_bridge  
7. Using remote computer to initial & set goal for the ddsbot through RVIZ  
8. Do the same thing on another ddsbots (in the same network),  
   once the ros1_bridge has been executed, all others ddsbots can be visualized on its own RVIZ.  


