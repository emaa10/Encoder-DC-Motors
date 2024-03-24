#include <Arduino.h>

   /*
  BTS7960-43A-Driver
  made on 22 Nov 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home
*/

#define LEFT_LPWM 5
#define LEFT_RPWM 4
#define RIGHT_LPWM 2
#define RIGHT_RPWM 3

int outLeft;
int outRight;

void setup() {
  Serial.begin(9600);
  pinMode(RIGHT_RPWM,OUTPUT);
  pinMode(RIGHT_LPWM,OUTPUT);

  pinMode(LEFT_RPWM,OUTPUT);
  pinMode(LEFT_LPWM,OUTPUT);

  outLeft = 100;
  outRight = 100;
  

}
 
 
void loop() {
  
  
  analogWrite(RIGHT_LPWM,outRight);
  analogWrite(RIGHT_RPWM,0);
  analogWrite(LEFT_LPWM,outLeft);
  analogWrite(LEFT_RPWM,0);
  delay(1000);

}