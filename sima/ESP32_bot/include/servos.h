#include <ESP32Servo.h>

#define servo1 15
#define servo2 2
#define servo3 5
#define servo4 4
#define servo5 16
#define servo6 17

Servo myservo;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;


void initialiseServos(){
    myservo.attach(15);
    myservo1.attach(2);
    myservo2.attach(5);
    myservo3.attach(4);
    myservo4.attach(16);
    myservo5.attach(17);
}