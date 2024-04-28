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

    //flag: stop, blue, yellow
    if(command == 'c') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();
      if(value == 0){
        flagLeft("out");
      } else if(value == 1){
        flagLeft("in");
      } else if(value == 2){
        flagRight("out");
      } else if(value == 3){
        flagRight("in");
      }
    }

    //belt
    if(command == 'b') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();
      
      if(value == 0){
        beltDrive("S_DOWN_P_DOWN");
      } else if(value == 1){
        beltDrive("S_MID_P_MID");
      } else if(value == 2){
        beltDrive("S_UP_P_DOWN");
      } else if(value == 3){
        beltDrive("S_UP_P_UP");
      }
    }

    //display: 0-999
    if(command == 'x') {
      String valueStr = input.substring(1);
      int displayCounter = valueStr.toInt();
      displayInteger(displayCounter);
    }

    //slotter/potter 
    if(command == 'u') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();

      if(value == 0){
        potter(1);
        slotterFront(1);
      } else if(value == 1){
        slotterFront(2);
        potter(2);
      } else if(value == 2){
        slotterFront(3);
        potter(2);
      } else if(value == 3){
        slotterFront(4);
        potter(3);
      }
    }

    //slotter back
    if(command == 'l') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();

      if(value == 0){
        slotterBack(1);
      } else if(value == 1){
        slotterBack(2);
      }else if(value == 2){
        slotterBack(3);
      }
    }

    //debug
    if(command == 'd') {
      String valueStr = input.substring(1);
      int value = valueStr.toInt();
      
      slotterFront(4);
      potter(3);
    }
  }
}

void setup(){
  // Init Serial Monitor
  Serial.begin(115200);

  while(!Serial);
  
  initSimas();
  initialiseDisplay();
  initialiseServos();
  initialiseStepper();
}

void loop(){
  getData();
}