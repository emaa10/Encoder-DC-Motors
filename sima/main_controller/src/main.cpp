#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "stepperDrive.h"
#include "display.h"
#include "esp_now_com.h"
#include "servos.h"

void getData() {
  if(Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    char command = input.charAt(0);

    //sima: blue, yellow
    if(command == 's') {
      String valueStr = input.substring(1);
      int color = valueStr.toInt();
      teamBlue = color==1?true:false;
      sendSimas();
    }
    //spinner: stop, blue, yellow
    if(command == 'c') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();
      spinnerState = value;
    }
    //belt: home, up, down
    if(command == 'b') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();
      
      beltState = value;
    }
    if(command == 'r') {
      beltPos = beltLimit;
    }
    //display: 0-999
    if(command == 'x') {
      String valueStr = input.substring(1);
      int displayCounter = valueStr.toInt();
      displayInteger(displayCounter);
    }
    //gripper up
    if(command == 'u') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();

      if(value == 0){
        myservo5.write(120);
      } else if(value == 1) {
        myservo5.write(35);
      } else if(value == 2) {
        myservo5.write(20);
      } else if(value == 3) {
        myservo5.write(15);
      }
    }
    //gripper down
    if(command == 'l') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();

      if(value == 0){
        myservo3.write(130);
        myservo4.write(25);
      } else if(value == 1) {
        myservo3.write(25);
        myservo4.write(130);
      }
    }
  }
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  while(!Serial);
  
  pinMode(limitSwitch, INPUT_PULLUP);


  initSimas();
  initialiseDisplay();
  initialiseServos();
  initialiseStepper();
}

void loop()
{
  getData();
  
  if(beltState == 1){
    beltDown();
  } else if(beltState == 2) {
    beltMiddle();
  } else if(beltState == 3) {
    beltUp();
  }

  if(spinnerState == 1){
    spinnerTurnLeft();
  } else if(spinnerState == 2) {
    spinnerTurnRight();
  }
}