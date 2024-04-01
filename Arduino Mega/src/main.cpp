/**
* Slave: PID Tuning, Odometry, driving -> return enc data for position calculation on raspberry pi
* Baudrate: 115200
*/

#include <Arduino.h>
#include "./pins.h"

volatile long int counterLEFT = 0; // This variable will increase or decrease depending on the rotation of encoder
volatile long int counterRIGHT = 0;
float currentPwmLeft;
float currentPwmRight;

float x=0; //muss mittelpunkt sein
float y=0;
float theta=0;
long int lastEncLeft=0; // maybe not needed
long int lastEncRight=0;

volatile long int encoderLeft=0;
volatile long int encoderRight=0;

template<typename T>
void print(const T& input) {
  Serial.print(input);
}
void print(const char* input) {
  Serial.print(input);
}
template<typename T>
void println(const T& input) {
  Serial.println(input);
}
void println(const char* input) {
  Serial.println(input);
}

// encoder functions: LEFT 1, left 2, right 1, right 2
void ai0() {
  if (digitalRead(LEFT_ENC_A_PHASE) == LOW) { counterLEFT++; }
  else { counterLEFT--; }
}

void ai1() {
  if (digitalRead(LEFT_ENC_B_PHASE) == LOW) { counterLEFT--; }
  else { counterLEFT++; }
}

void bi0() {
  if (digitalRead(RIGHT_ENC_A_PHASE) == LOW) { counterRIGHT++; }
  else { counterRIGHT--; }
}

void bi1() {
  if (digitalRead(RIGHT_ENC_B_PHASE) == LOW) { counterRIGHT--; }
  else { counterRIGHT++; }
}

void setup()
{
  Serial.begin(115200);
  // encoder
  pinMode(LEFT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PHASE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PHASE), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PHASE), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PHASE), bi0, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PHASE), bi1, RISING);

  // dc
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
}

void loop()
{
  // return x and y here
  delay(5);
}