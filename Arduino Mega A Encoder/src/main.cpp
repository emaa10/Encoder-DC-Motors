#include <Arduino.h>
volatile unsigned int counterLEFT = 0; // This variable will increase or decrease depending on the rotation of encoder
volatile unsigned int counterRIGHT = 0; // This variable will increase or decrease depending on the rotation of encoder

// Encoder pins
const int LEFT_ENC_A_PHASE = 3;
const int LEFT_ENC_B_PHASE = 2;
const int RIGHT_ENC_A_PHASE = 18;
const int RIGHT_ENC_B_PHASE = 19;

void ai0()
{
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW)
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void ai1()
{
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

void setup()
{
  Serial.begin(9600);

  pinMode(3, INPUT_PULLUP); // internal pullup input pin 2

  pinMode(2, INPUT_PULLUP); // internalเป็น pullup input pin 3
                            // Setting up interrupt
  // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);

  // B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}

void loop()
{
  Serial.println(counter);
  delay(20);
}