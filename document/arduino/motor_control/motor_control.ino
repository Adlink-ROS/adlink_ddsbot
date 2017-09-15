/*
Copyright 2017 ADLINK Technology Inc.
Developer: Chester, Tseng
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

const int out1_pin = 5;
const int out2_pin = 6;
const int out3_pin = 9;
const int out4_pin = 10;
const int decoder_pin_1 = 2;
const int decoder_pin_2 = 3;
volatile int decoder_pin_1_counter = 0;
volatile int decoder_pin_2_counter = 0;

volatile int decoder_pin_1_delay_counter = 0;
volatile int decoder_pin_2_delay_counter = 0;

// Report internal;
const int report_period = 1000;


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
  
 // Set 2 external interrupt pin to input and attach it's ISR
  pinMode(decoder_pin_1, INPUT);
  pinMode(decoder_pin_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(decoder_pin_1), decoder_1_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(decoder_pin_2), decoder_2_isr, FALLING);

  // Start timer for reporting wheel speed
  MsTimer2::set(report_period, report_decoder_result_isr); // 100ms period
  MsTimer2::start();
}

void loop() {
    if (Serial.available()) {
      String commad_string = Serial.readString();
      String out_1 = getValue(commad_string, ',', 0);
      String out_2 = getValue(commad_string, ',', 1);
      String out_3 = getValue(commad_string, ',', 2);
      String out_4 = getValue(commad_string, ',', 3);
      if (out_1 != NULL) {
        int out1_value = out_1.toFloat();
        analogWrite(out1_pin, convert_adc(out1_value));
      }
      if (out_2 != NULL) {
        int out2_value = out_2.toFloat();
        analogWrite(out2_pin, convert_adc(out2_value));
      }
      if (out_3 != NULL) {
        int out3_value = out_3.toFloat();
        analogWrite(out3_pin, convert_adc(out3_value));
      }
      if (out_4 != NULL) {
        int out4_value = out_4.toFloat();
        analogWrite(out4_pin, convert_adc(out4_value));
      }
    }
}

void decoder_1_isr() {
  decoder_pin_1_counter += 1;
}

void decoder_2_isr() {
  decoder_pin_2_counter += 1;
}

void report_decoder_result_isr() {
  // This is a workaround to prevent from motor idle when speed  = 0;
  if (decoder_pin_1_delay_counter > 50) {
    analogWrite(out3_pin, 0);
    analogWrite(out4_pin, 0);
    decoder_pin_1_delay_counter = 0;
  }
  
  if (decoder_pin_2_delay_counter > 50) {
    analogWrite(out1_pin, 0);
    analogWrite(out2_pin, 0);
    decoder_pin_2_delay_counter = 0;
  }
  
  char str[16];
  sprintf(str, "%d,%d", decoder_pin_1_counter*18, decoder_pin_2_counter*18); // --> 360deg/20pic = 18
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

// convert required deg/s to pwm command (feedforward)
float convert_adc(int value) {
  return 0.1335 * value + 7.07;  
}



