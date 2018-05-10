# ADLink DDSBot (ROS 1.0/2.0 based swarm robots architecture)

## Abstract

Swarm robots using Opensplice DDS with ROS 1&2  
[Official Slides] <https://github.com/Adlink-ROS/adlink_ddsbot/blob/release-1.0/document/Next-Gen_Smart_Manufacturing.pdf>  
[Youtube Video] <https://www.youtube.com/watch?v=7ZIDkLKD4Y8>  
[![alt text](http://img.youtube.com/vi/7ZIDkLKD4Y8/0.jpg)](https://www.youtube.com/watch?v=7ZIDkLKD4Y8)

## Developer & Team

HaoChih Lin (haochih.lin@adlinktech.com)  
Chester Tseng (chester.tseng@adlinktech.com)  
Erik Boasson (erik.boasson@adlinktech.com)  
Ryan Chen (ryanjb.chen@adlinktech.com)

ADLINK Technology, Inc  
Advanced Robotic Platform Group

## License

Apache 2.0  
Copyright 2017 ADLINK Technology, Inc.

## Software Architecture

![alt text](https://github.com/Adlink-ROS/adlink_ddsbot/blob/release-1.0/document/adlink_ddsbot_softarch.jpg)

## Hardware Architecture

![alt text](https://github.com/Adlink-ROS/adlink_ddsbot/blob/release-1.0/document/adlink_ddsbot_hardarch.jpg)  
Build your own low-cost ddsbot (~250USD)  
[Assembling PDF] [adlink_ddsbot/document/hardware/adlink_ddsbot_hardware.pdf](https://github.com/Adlink-ROS/adlink_ddsbot/blob/release-1.0/document/hardware/adlink_ddsbot_hardware.pdf)

[Arduino firmware]  
For UNO (motor control): adlink_ddsbot/document/arduino/motor_control/motor_control.ino  
For Nano (GY85 IMU): adlink_ddsbot/document/arduino/Razor_AHRS/Razor_AHRS.ino

## Tutorial

### System prerequisite

[Desktop]

* ROS 1.0
  (Kinetic: <http://wiki.ros.org/kinetic/Installation/Ubuntu)>
* ROS 2.0 (tested on Ardent)
* create your own catkin_ws
  (<http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)>
* Install required dependencies:  
  `$ sudo apt-get install git ros-kinetic-navigation ros-kinetic-geographic-msgs ros-kinetic-slam-gmapping ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d ros-kinetic-robot-localization ros-kinetic-stage* -y`
* `git clone adlink_ddsbot to your catkin_ws/src and compile`

### Simulation (stage, two computers)

1. Check the hostname (should be unique in the network)
2. Check the system time synchronization ($ sudo ntpdate IP_TARGET)
3. Open a terminal (remember to source ros2 & ros1 setup.bash)
4. `$ export HOSTNAME`
5. `$ roslaunch adlink_ddsbot Swarm_Sim_Stage.launch`
6. `$ ros2 run ros1_bridge parameter_bridge`
7. Repeat the same steps on another computer/robot
    (both should be in the same network)
8. Then, each robot can be aware of all the others shown on the rviz.

### Real robot (raspberryPi3)

[Image files]
Download pre-built image for Pi3 (ubuntu mate 16.04, ros 1&2, 13GB)  
link: <https://drive.google.com/open?id=0BxI3PXhTd_3HWmh2c2VSbWFNRHM>  
Extract the file from .gz file:
`$ gunzip -c ADLINK_DDSBot_Pi3_20171002.img.gz > ADLINK_DDSBot_Pi3_20171002.img`
Burn image file to SD card

[Setup steps]

1. Connect Pi3 with your desktop by either ethernet cable or debug cable
2. Power-on and login: (passward is: adlinkros)
   * hdmi display: you can see the ADLink logo on the desktop
   * ethernet: the default id address of provided image is 10.0.0.1 (`$ ssh pi@10.0.0.1`)
   * debug cable: enter the password
3. Change robot's hostname and connect to local network
4. Synchronize the system time through AP server/specific computer
    (`$ sudo ntpdate IP_ADDRESS`)
5. Change the ROS 1 multi-machines variables in .bashrc
    (for each robot, it has own roscore)
6. For Single robot test: `$ roslaunch adlink_ddsbot Adlink_DDSBot_Nav_Single.launch`
7. For Swarm robots demo:
   * `$ roslaunch adlink_ddsbot Adlink_DDSBot_Nav_Swarm.launch`
   * `$ ros2 run ros1_bridge parameter_bridge`
8. Using remote computer to initial & set goal for the ddsbot through RVIZ
9. Do the same thing on another ddsbots (in the same network),
    once the ros1_bridge has been executed, all others ddsbots can be visualized on RVIZ.

[Node Graph]
![alt text](https://github.com/Adlink-ROS/adlink_ddsbot/blob/release-1.0/document/adlink_ddsbot_nodes.png)
