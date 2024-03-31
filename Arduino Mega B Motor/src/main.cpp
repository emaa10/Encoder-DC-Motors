// nur pwm getten und auf motor packen
#include <Arduino.h>

// left dc motor pins
#define LEFT_LPWM 5
#define LEFT_RPWM 4

// right dc motor pins
#define RIGHT_LPWM 2
#define RIGHT_RPWM 3

float pwmLeft=0; // default 0
float pwmRight=0;

float receivedValue1; // pwm value left
float receivedValue2; // pwm value right

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    receivedValue1 = Serial.parseFloat();
    while (Serial.read() != ',');
    receivedValue2 = Serial.parseFloat();
    while (Serial.read() != '\n');
    
    Serial.print("Received value 1: ");
    Serial.println(receivedValue1);
    Serial.print("Received value 2: ");
    Serial.println(receivedValue2);
  }
  pwmLeft = abs(receivedValue1);
  pwmRight = abs(receivedValue2);

  if(receivedValue1 < 0) { // wenn wir links rückwärts fahren wollen
    analogWrite(LEFT_LPWM, 0);
    analogWrite(LEFT_RPWM, pwmLeft);
  } else{ // wenn wir vorwärts fahren wollen
    analogWrite(LEFT_LPWM, pwmLeft);
    analogWrite(LEFT_RPWM, 0);
  }

  if(receivedValue2 < 0) {  // wenn wir rechts rückwärts fahren wollen
    analogWrite(RIGHT_LPWM, 0);
    analogWrite(RIGHT_RPWM, pwmRight);
  } else{
    analogWrite(RIGHT_LPWM, pwmRight);
    analogWrite(RIGHT_RPWM, 0);
  }

  delay(10);
}