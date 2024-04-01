/**
* Slave: PID Tuning, Odometry, driving -> return enc data for position calculation on raspberry pi
* Baudrate: 115200
*/

#include <Arduino.h>
#include "./main.h"


float currentPwmLeft;
float currentPwmRight;

float x=0; //muss mittelpunkt sein
float y=0;
float theta=0;
long int lastEncLeft=0; // maybe not needed
long int lastEncRight=0;

volatile long int encoderLeft=0;
volatile long int encoderRight=0;

long int leftEncoderChange;
long int rightEncoderChange;

int counter=0;

long int oldEncoderLeft;
long int oldEncoderRight;

template<typename T>
void print(const T& input) {
  Serial.print(input);
}
void print(const char* input) {
  Serial.print(input);
}
template<typename T>
void println(const T& input) {
  Serial.println(input);
}
void println(const char* input) {
  Serial.println(input);
}

// encoder functions: LEFT 1, left 2, right 1, right 2
void ai0() {
  if (digitalRead(LEFT_ENC_A_PHASE) == LOW) { encoderLeft++; }
  else { encoderLeft--; }
}

void ai1() {
  if (digitalRead(LEFT_ENC_B_PHASE) == LOW) { encoderLeft--; }
  else { encoderLeft++; }
}

void bi0() {
  if (digitalRead(RIGHT_ENC_A_PHASE) == LOW) { encoderRight++; }
  else { encoderRight--; }
}

void bi1() {
  if (digitalRead(RIGHT_ENC_B_PHASE) == LOW) { encoderRight--; }
  else { encoderRight++; }
}

bool pullCordConnected() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    if (message == "p,0") {
      return false;
    } else if (message == "p,1") {
      return true;
    }
  }
  return true; // Standardmäßig false zurückgeben, wenn keine passende Nachricht empfangen wurde
}

long int getEncoderLeft() {
  return encoderLeft;
}

long int getEncoderRight() {
  return encoderRight;
}

void stopMotor() {sendPwmValues(0, 0); }

void setEncoderZero() {encoderLeft=0; encoderRight=0;}

float getAngle(float input = theta) {
    float result = theta*180/M_PI;
    // result = fmod((result + 360.0), 360.0);
    // now in updatepos with theta
    return result;
}

void setPwmValues(float pwmLeft, float pwmRight) {
  pL = abs(pwmLeft);
  pR = abs(pwmRight);
  currentPwmLeft=pwmLeft;
  currentPwmRight=pwmRight;

  if(pwmLeft < 0) { // wenn wir links rückwärts fahren wollen
    analogWrite(LEFT_LPWM, 0);
    analogWrite(LEFT_RPWM, pL);
  } else{ // wenn wir vorwärts fahren wollen
    analogWrite(LEFT_LPWM, pL);
    analogWrite(LEFT_RPWM, 0);
  }

  if(pwmRight < 0) {  // wenn wir rechts rückwärts fahren wollen
    analogWrite(RIGHT_LPWM, 0);
    analogWrite(RIGHT_RPWM, pR);
  } else{
    analogWrite(RIGHT_LPWM, pR);
    analogWrite(RIGHT_RPWM, 0);
  }
}

void drive(float drivePwmLeft, float drivePwmRight) {
    if(drivePwmLeft > 150) {drivePwmLeft = 150;}
    if(drivePwmRight > 150) {drivePwmRight = 150;}
    if(drivePwmLeft < -150) {drivePwmLeft = -150;}
    if(drivePwmRight < -150) {drivePwmRight = -150;}
    setPwmValues(drivePwmLeft, drivePwmRight);
};

void updatePosition(float leftEncChange, float rightEncChange) {
    float leftDistance = leftEncChange / pulsesPerMM;
    float rightDistance = rightEncChange / pulsesPerMM;
    float distance = (leftDistance + rightDistance) / 2;
    float dTheta = (rightDistance - leftDistance) / wheelDistance;
    x += distance * cos(theta + dTheta / 2);
    y += distance * sin(theta + dTheta / 2);
    theta += dTheta;
    theta = fmod((theta + 2 * M_PI), (2 * M_PI)); // test in radian
    // std::cout << "X: " << x << " Y: " << y << " Theta: " << getAngle() << std::endl;
}

void updatePositionThread() { // NEED MILLIS
    while(1) {
        float leftEnc1 = getEncoderLeft();
        float rightEnc1 = getEncoderRight();
        delay(1000);
        float leftEnc2 = getEncoderLeft();
        float rightEnc2 = getEncoderRight();
        updatePosition(leftEnc2 - leftEnc1, rightEnc2 - rightEnc1);
    }
}



void setup()
{
  Serial.begin(115200);
  // encoder
  pinMode(LEFT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PHASE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PHASE), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PHASE), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PHASE), bi0, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PHASE), bi1, RISING);

  // dc
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
}

void loop()
{
  // return x and y here
  delay(5);
}