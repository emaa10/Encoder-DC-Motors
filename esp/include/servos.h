#include <ESP32Servo.h>

#define servo1 15
#define servoPotterPin 2
#define servoSlotterFrontPin 5
#define servoFlagRightPin 4
#define servoFlagLeftPin 16
#define servoSlotterBackPin 17

Servo myservo;
Servo servoPotter;
Servo servoSlotterFront;
Servo servoFlagRight;
Servo servoFlagLeft;
Servo servoSlotterBack;


void initialiseServos(){
    myservo.attach(15);
    servoPotter.attach(servoPotterPin);
    servoSlotterFront.attach(servoSlotterFrontPin);
    servoFlagRight.attach(servoFlagRightPin);
    servoFlagLeft.attach(servoFlagLeftPin);
    servoSlotterBack.attach(servoSlotterBackPin);
}

void slotterFront(int input){
    if(input == 1){
        servoSlotterFront.write(122);    //all the way down slight negative tilt
    } else if(input == 2){
        servoSlotterFront.write(116);    //parallel to ground
    } else if(input == 3){
        servoSlotterFront.write(105);    //upwards tilted
    } else if(input == 4){
        servoSlotterFront.write(20);    //90°
    }
}

void slotterBack(int input){
    if(input == 1){
        servoSlotterBack.write(37);    //all the way down slight negative tilt
    } else if(input == 2){
        servoSlotterBack.write(100);    //upwards tilted
    } else if(input == 3){
        servoSlotterBack.write(132);    //90°
    }
}

void potter(int input){
    if(input == 1){
        servoPotter.write(80);    //all the way down slight negative tilt
    } else if(input == 2){
        servoPotter.write(94);    //parallel to ground
    } else if(input == 3){
        servoPotter.write(185);    //90°
    }
}

void flagLeft(String input){
    if(input == "out"){
        servoFlagLeft.write(50);
    } else if(input == "in"){
        servoFlagLeft.write(180);
    }
}

void flagRight(String input){
    if(input == "out"){
        servoFlagRight.write(160);
    } else if(input == "in"){
        servoFlagRight.write(50);
    }
}