#include <Arduino.h>

// Encoder pins
const int LEFT_ENC_A_PHASE = 3;
const int LEFT_ENC_B_PHASE = 2;
const int RIGHT_ENC_A_PHASE = 18;
const int RIGHT_ENC_B_PHASE = 19;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B_PHASE) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B_PHASE) == HIGH) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

void setup() {
  pinMode(LEFT_ENC_A_PHASE, INPUT);
  pinMode(LEFT_ENC_B_PHASE, INPUT);
  pinMode(RIGHT_ENC_A_PHASE, INPUT);
  pinMode(RIGHT_ENC_B_PHASE, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PHASE), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PHASE), rightEncoderISR, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // Send encoder counts to Raspberry Pi
  Serial.print("LeftEncoder:");
  Serial.print(leftEncoderCount);
  Serial.print(",RightEncoder:");
  Serial.println(rightEncoderCount);
  delay(100); // Adjust delay as needed
}
