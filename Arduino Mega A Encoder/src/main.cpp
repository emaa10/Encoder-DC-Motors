#include <Arduino.h>
volatile unsigned int counterLEFT = 0; // This variable will increase or decrease depending on the rotation of encoder
volatile unsigned int counterRIGHT = 0;

// Encoder pins
const int LEFT_ENC_A_PHASE = 18;
const int LEFT_ENC_B_PHASE = 19;
const int RIGHT_ENC_A_PHASE = 2;
const int RIGHT_ENC_B_PHASE = 3;

// LEFT_1
void ai0()
{
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(LEFT_ENC_A_PHASE) == LOW)
  {
    counterLEFT++;
  }
  else
  {
    counterLEFT--;
  }
}

// LEFT_2
void ai1()
{
  if (digitalRead(LEFT_ENC_B_PHASE) == LOW)
  {
    counterLEFT--;
  }
  else
  {
    counterLEFT++;
  }
}

// RIGHT_1
void bi0()
{
  if (digitalRead(RIGHT_ENC_A_PHASE) == LOW)
  {
    counterRIGHT++;
  }
  else
  {
    counterRIGHT--;
  }
}

// RIGHT_2
void bi1()
{
  if (digitalRead(RIGHT_ENC_B_PHASE) == LOW)
  {
    counterRIGHT--;
  }
  else
  {
    counterRIGHT++;
  }
}

void setup()
{
  Serial.begin(9600);

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
  // Serial.println(digitalPinToInterrupt(RIGHT_ENC_B_PHASE));
  Serial.print(counterLEFT);
  Serial.print(",");
  Serial.println(counterRIGHT);
  delay(20);
}