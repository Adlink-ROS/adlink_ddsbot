/*
Copyright 2017 ADLINK Technology Inc.
Developer: Chester, Tseng 
           HaoChih, LIN
Email: chester.tseng@adlinktech.com
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
#include <MsTimer2.h>

// Pin define
const int out1_pin = 5;
const int out2_pin = 6;
const int out3_pin = 9;
const int out4_pin = 10;
const int decoder_pin_1 = 3;
const int decoder_pin_2 = 2;

// Timer internal
const int timer_period = 50; // ms 
const float timer_hz = 1000.0/( (float)timer_period ); // hz

// encoder
volatile int decoder_pin_1_counter = 0;
volatile int decoder_pin_2_counter = 0;
volatile int decoder_pin_1_delay_counter = 0;
volatile int decoder_pin_2_delay_counter = 0;
volatile int encoder_res = 18; // 20 pulse in a circle --> 360/20 = 18(deg/pulse)

// controller
volatile float WL_ref = 0.0; //reference speed for left wheel (deg/sec) 
volatile float WR_ref = 0.0;
volatile float Kp_L = 0.1;
volatile float Kp_R = 0.1;

void setup() {
  // Set 4 pwm channel pin to output
  pinMode(out1_pin, OUTPUT);
  pinMode(out2_pin, OUTPUT);
  pinMode(out3_pin, OUTPUT);
  pinMode(out4_pin, OUTPUT);

  // Set 4 PWM channel pin to duty cycle: 0
  analogWrite(out1_pin, 0);
  analogWrite(out2_pin, 0);
  analogWrite(out3_pin, 0);
  analogWrite(out4_pin, 0);

  // Start serial port
  Serial.begin(115200);
  while (!Serial) {
  }
  Serial.setTimeout(20);
  
 // Set 2 external interrupt pin to input and attach it's ISR
  pinMode(decoder_pin_1, INPUT);
  pinMode(decoder_pin_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(decoder_pin_1), decoder_1_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(decoder_pin_2), decoder_2_isr, FALLING);

  // Start timer for motor control & reporting wheel speed 
  MsTimer2::set(timer_period, controller_repoter_isr); 
  MsTimer2::start();
}

void loop() {
    if (Serial.available()) {
      String commad_string = Serial.readString();
      String out_1 = getValue(commad_string, ',', 0); //getValue func should be removed (TBD) 
      String out_2 = getValue(commad_string, ',', 1);
      String out_3 = getValue(commad_string, ',', 2);
      String out_4 = getValue(commad_string, ',', 3);
      if (out_1 != NULL)
        WL_ref =  out_1.toFloat(); // deg/sec
        
      if (out_2 != NULL)
        WL_ref = -out_2.toFloat();
         
      if (out_3 != NULL)
        WR_ref =  out_3.toFloat();
        
      if (out_4 != NULL)
        WR_ref = -out_4.toFloat();
    }
}

void decoder_1_isr() {
  decoder_pin_1_counter += 1;
}

void decoder_2_isr() {
  decoder_pin_2_counter += 1;
}

// convert required deg/s to pwm command (based on statistical)
float feedforward(float value) {
  return 0.1335 * value + 7.07;  
}

void controller_repoter_isr() {
  // controller (feedforward + feedback)
  float cmd = 0.0;
  float error = 0.0;
  if(WL_ref >= 0.0)
  {
    error = WL_ref - (float)(decoder_pin_1_counter*encoder_res)*timer_hz; // deg/sec
    cmd = feedforward(WL_ref) + error*Kp_L;
    if(cmd>=0.0)
    {
      analogWrite(out1_pin, (int)cmd);
      analogWrite(out2_pin, 0);
    }
    else
    {
      analogWrite(out1_pin, 0);
      analogWrite(out2_pin, -(int)cmd);
    }
  }
  else // the encoder has no capability of distinguishing the cw/ccw rotation
  {
    error = -WL_ref - (float)(decoder_pin_1_counter*encoder_res)*timer_hz; // deg/sec
    cmd = feedforward(WL_ref) + error*Kp_L;
    if(cmd>=0.0)
    {
      analogWrite(out2_pin, (int)cmd);
      analogWrite(out1_pin, 0);
    }
    else
    {
      analogWrite(out2_pin, 0);
      analogWrite(out1_pin, -(int)cmd);
    }
  }

  if(WR_ref >= 0.0)
  {
    error = WR_ref - (float)(decoder_pin_2_counter*encoder_res)*timer_hz; // deg/sec
    cmd = feedforward(WR_ref) + error*Kp_R;
    if(cmd>=0.0)
    {
      analogWrite(out3_pin, (int)cmd);
      analogWrite(out4_pin, 0);
    }
    else
    {
      analogWrite(out3_pin, 0);
      analogWrite(out4_pin, -(int)cmd);
    }
  }
  else // the encoder has no capability of distinguishing the cw/ccw rotation
  {
    error = -WR_ref - (float)(decoder_pin_2_counter*encoder_res)*timer_hz; // deg/sec
    cmd = feedforward(WR_ref) + error*Kp_R;
    if(cmd>=0.0)
    {
      analogWrite(out4_pin, (int)cmd);
      analogWrite(out3_pin, 0);
    }
    else
    {
      analogWrite(out4_pin, 0);
      analogWrite(out3_pin, -(int)cmd);
    }
  }
  
  
  // This is a workaround to prevent from motor idle when speed  = 0;
  if (decoder_pin_1_delay_counter > (100 + WR_ref/encoder_res/timer_hz) ) {
    WR_ref = 0.0;
    analogWrite(out3_pin, 0);
    analogWrite(out4_pin, 0);
    decoder_pin_1_delay_counter = 0;
  }
  
  if (decoder_pin_2_delay_counter > (100 + WL_ref/encoder_res/timer_hz) ) {
    WL_ref = 0.0;
    analogWrite(out1_pin, 0);
    analogWrite(out2_pin, 0);
    decoder_pin_2_delay_counter = 0;
  }

  // report encoder info (deg/sec)
  char str[16];
  sprintf(str, "%d,%d\r\n", decoder_pin_1_counter*encoder_res
                          , decoder_pin_2_counter*encoder_res ); // deg/sec
  Serial.print(str);

  if (decoder_pin_1_counter == 0)
    decoder_pin_1_delay_counter += 1;
  if (decoder_pin_2_counter == 0)
    decoder_pin_2_delay_counter += 1;
  
  decoder_pin_1_counter = 0;
  decoder_pin_2_counter = 0;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


