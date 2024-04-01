/**
* Slave: PID Tuning, Odometry, driving -> return enc data for position calculation on raspberry pi
*/

#include <Arduino.h>
#include "./pins.h"

volatile long int counterLEFT = 0; // This variable will increase or decrease depending on the rotation of encoder
volatile long int counterRIGHT = 0;
float currentPwmLeft;
float currentPwmRight;

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

  pinMode(LEFT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PHASE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PHASE), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PHASE), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PHASE), bi0, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PHASE), bi1, RISING);
}

void loop()
{
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte == '0') {
      counterRIGHT = 0;
      counterLEFT = 0;
    }
  }

  // Serial.println(digitalPinToInterrupt(RIGHT_ENC_B_PHASE));
  Serial.print(counterLEFT);
  Serial.print(",");
  Serial.println(counterRIGHT);
  delay(5);
}