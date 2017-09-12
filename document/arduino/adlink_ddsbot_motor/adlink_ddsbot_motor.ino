/*
Copyright 2017 ADLINK Technology Inc.
Developer: HaoChih, LIN
Email: haochih.lin@adlinktech.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <SoftwareSerial.h>

#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle  nh;

int motorIn1 = 5;
int motorIn2 = 6;
int motorIn3 = 9;
int motorIn4 = 10;

float motorScaleL = 510.0; // assume 0.5 m/s = 255
float motorScaleR = 510.0;
float wheelWidth = 0.15; // unit: m
float wheelRadius = 0.035; // unit: m
float Vx = 0.0;
float Vyaw = 0.0;
float VR = 0.0;
float VL = 0.0;
int motorOut1 = 0;
int motorOut2 = 0;
int motorOut3 = 0;
int motorOut4 = 0;

int motorMax = 255; // for pwm

/*
Stop: 0 0 0 0
Backward: 100 0 100 0
Forward: 0 100 0 100
Right: 0 0 0 100
Left: 0 100 0 0

Overall: Right v.s Left
*/
void motor_pwm(int In1, int In2, int In3, int In4)
{
  if(In1 > motorMax)
    In1 = motorMax;
  if(In1 <= 0)
    In1 = 0;
  
  if(In2 > motorMax)
    In2 = motorMax;
  if(In2 <= 0)
    In2 = 0;
  
  if(In3 > motorMax)
    In3 = motorMax;
  if(In3 <= 0)
    In3 = 0;
   
  if(In4 > motorMax)
    In4 = motorMax;
  if(In4 <= 0)
    In4 = 0;  
    
  analogWrite(motorIn1, In1);
  analogWrite(motorIn2, In2);
  analogWrite(motorIn3, In3);
  analogWrite(motorIn4, In4);
}

void steering( const geometry_msgs::Twist& cmd_msg)
{
  
  Vx   = -float(cmd_msg.linear.x); // unit: m/s
  Vyaw = -float(cmd_msg.angular.z)*1.5; // unit: rad/s, factor 1.5 is bias 
  
  VR = Vx + wheelWidth/2.0*Vyaw;
  VL = Vx - wheelWidth/2.0*Vyaw;
  
  if(VR >= 0)
  {
    motorOut1 = 0;
    motorOut2 = int(VR*motorScaleR); 
  }
  else
  {
    motorOut1 = -int(VR*motorScaleR);
    motorOut2 = 0; 
  }
  
  if(VL >= 0)
  {
    motorOut3 = 0;
    motorOut4 = int(VL*motorScaleL); 
  }
  else
  {
    motorOut3 = -int(VL*motorScaleL);
    motorOut4 = 0; 
  }
  
  motor_pwm(motorOut1, motorOut2, motorOut3, motorOut4);
  digitalWrite(13, !digitalRead(13));  //toggle led  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", steering);

void setup()
{
  Serial.begin(57600);
  
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  pinMode(13, OUTPUT);
  motor_pwm(0, 0, 0, 0); // Force STOP
  
  nh.initNode();
  nh.subscribe(sub);
  delay(30);
}

void loop()
{
  nh.spinOnce();
  delay(0.1);
}
