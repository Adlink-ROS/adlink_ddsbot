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
#include <ArduinoJson.h>
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
const int report_period = 100;


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
    DynamicJsonBuffer jsonBuffer;
    if (Serial.available()) {
      String commad_string = Serial.readString();
      JsonObject& root = jsonBuffer.parseObject(commad_string);
      //Serial.println(commad_string);
      String out_1 = root["out_1"];
      String out_2 = root["out_2"];
      String out_3 = root["out_3"];
      String out_4 = root["out_4"];
      if (out_1 != NULL) {
        int out1_value = root["out_1"];
        analogWrite(out1_pin, out1_value);
        //Serial.println(out1_value);
      }
      if (out_2 != NULL) {
        int out2_value = root["out_2"];
        analogWrite(out2_pin, out2_value);
        //Serial.println(out2_value);
      }
      if (out_3 != NULL) {
        int out3_value = root["out_3"];
        analogWrite(out3_pin, out3_value);
        //Serial.println(out3_value);
      }
      if (out_4 != NULL) {
        int out4_value = root["out_4"];
        analogWrite(out4_pin, out4_value);
        //Serial.println(out4_value);
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
  if ((decoder_pin_1_counter == 0) && (decoder_pin_1_delay_counter > 10)) {
    analogWrite(out1_pin, 0);
    analogWrite(out2_pin, 0);
    decoder_pin_1_delay_counter = 0;
  }
  
  if ((decoder_pin_2_counter == 0) && (decoder_pin_2_delay_counter > 10)) {
    analogWrite(out3_pin, 0);
    analogWrite(out4_pin, 0);
    decoder_pin_2_delay_counter = 0;
  }
  
  char str[16];
  sprintf(str, "%d,%d\r\n", decoder_pin_1_counter, decoder_pin_2_counter);
  Serial.print(str);
  decoder_pin_1_counter = 0;
  decoder_pin_2_counter = 0;

  decoder_pin_1_delay_counter += 1;
  decoder_pin_2_delay_counter += 1;
}

