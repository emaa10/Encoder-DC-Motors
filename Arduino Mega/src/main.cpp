#include "simplepid.h"
#include <Arduino.h>
#include <util/atomic.h>

// Define pins

#define LEFT_ENC_A_PHASE 18
#define LEFT_ENC_B_PHASE 19
#define RIGHT_ENC_A_PHASE 2
#define RIGHT_ENC_B_PHASE 3

const int lpwm[] = {9, 11};
const int rpwm[] = {8, 10};
String DEBUG = "";

// Define Globals

#define NMOTORS 2
#define pwmCutoff 12 // Set minimum drivable pwm value
long prevT = 0;
volatile int posi[] = {0, 0};
int lastPos[] = {0, 0};
SimplePID pid[NMOTORS];
int target[] = {0, 0};

const float pulsesPerEncRev = 1200;
const float encWheelDiameterCM = 5;
const float motorWheelDiameterCM = 7;
const float encWheelScope = encWheelDiameterCM * M_PI;
const float motorWheelScope =
    motorWheelDiameterCM * M_PI; // distance travelled per rev
const float pulsesPerRev = pulsesPerEncRev * (motorWheelScope / encWheelScope);
const float pulsesPerMM = pulsesPerRev / motorWheelScope / 10;
const float pulsesPerCM = pulsesPerRev / motorWheelScope;
const float pwmSpeed = 100; // default pwm speed
const float pulsesPerSec =
    pulsesPerRev; // goal pulses per sec 1680, 1 round per second
const float wheelDistance =
    128; // abstand der encoderräder in mm, muss vllt geändert werden
const float wheelDistanceBig = 204; // in mm, muss vllt geändert werden
const float turnValue =
    wheelDistance * M_PI / 360; // abstand beider räder um 1° zu fahren

float x = 225;
float y = 225;
float theta = 0;
bool isDriving = false;

long lastPosUpdate;
bool leftTriggered = true;
bool rightTriggered = true;
// Encoder read functions

void ai0() {
  if (digitalRead(LEFT_ENC_A_PHASE) == LOW) {
    posi[1]++;
  } else {
    posi[1]--;
  }
}

void ai1() {
  if (digitalRead(LEFT_ENC_B_PHASE) == LOW) {
    posi[1]--;
  } else {
    posi[1]++;
  }
}

void bi0() {
  if (digitalRead(RIGHT_ENC_A_PHASE) == LOW) {
    posi[0]++;
  } else {
    posi[0]--;
  }
}

void bi1() {
  if (digitalRead(RIGHT_ENC_B_PHASE) == LOW) {
    posi[0]--;
  } else {
    posi[0]++;
  }
}

// PID program functions

void setMotor(int dir, int pwmVal, int lpwm, int rpwm) {
  if (dir == 1) {
    analogWrite(lpwm, pwmVal);
  } else if (dir == -1) {
    analogWrite(rpwm, pwmVal);
  } else {
    analogWrite(lpwm, 0);
    analogWrite(rpwm, 0);
  }
}

void updatePosition(float leftEncChange, float rightEncChange);
void resetPosition() {
  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NMOTORS; k++) {
      pos[k] = posi[k];
    }
  }
  // Update Changed Position
  updatePosition(pos[0] - lastPos[0], pos[1] - lastPos[1]);

  lastPos[0] = 0;
  lastPos[1] = 0;
  posi[0] = 0;
  posi[1] = 0;
  target[0] = 0;
  target[1] = 0;
}

// Driving functions

void driveDistance(int distance) {
  resetPosition();

  target[0] = 7.639437 * distance;
  target[1] = 7.639437 * distance;
}

void turnAngle(int degree) {
  resetPosition();

  int distance = 128 * 3.1415926 / 360 * degree;
  target[0] = -7.639437 * distance;
  target[1] = 7.639437 * distance;
}

// Serial Communication

void getData() { // get the data and run the actions
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    char command = input.charAt(0);

    if (command == 's') {
      resetPosition();
    } else if (command == 'd') {
      String valueStr = input.substring(2);
      int distance = valueStr.toInt();
      driveDistance(distance);
    } else if (command == 'w') {
      leftTriggered = false;
      rightTriggered = false;
      driveDistance(3000);
    } else if (command == 't') {
      String valueStr = input.substring(2);
      float angle = valueStr.toFloat();
      turnAngle(angle);
    }
  }
}

void sendData() {
  String data;
  data += isDriving ? "d" : "s";
  data += "x";
  data += String(x);
  data += "y";
  data += String(y);
  data += "t";
  data += String(theta);
  data += " ";
  data += DEBUG;
  Serial.println(data);
}

// Position update function

void updatePosition(float leftEncChange, float rightEncChange) {
  float leftDistance = leftEncChange / pulsesPerMM;
  float rightDistance = rightEncChange / pulsesPerMM;
  float distance = (leftDistance + rightDistance) / 2;
  float dTheta = (rightDistance - leftDistance) / wheelDistance;
  x += distance * cos(theta + dTheta / 2);
  y += distance * sin(theta + dTheta / 2);
  theta += dTheta;
  theta = fmod((theta + 2 * M_PI), (2 * M_PI)); // test in radian
  sendData();
}

// Setup function

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PHASE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PHASE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PHASE), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PHASE), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PHASE), bi0, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PHASE), bi1, RISING);

  target[0] = 0;
  target[1] = 0;

  for (int k = 0; k < NMOTORS; k++) {
    pid[k].setParams(1, 0, 0, 100);
  }

  lastPosUpdate = micros();
}

// Loop function

void loop() {
  getData();
  DEBUG = "";
  DEBUG += "posi 0: ";
  DEBUG += posi[0];
  DEBUG += " posi 1: ";
  DEBUG += posi[1];
  DEBUG += " target 0: ";
  DEBUG += target[0];
  DEBUG += " target 1: ";
  DEBUG += target[1];

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NMOTORS; k++) {
      pos[k] = posi[k];
    }
  }

  // Update Changed Position
  // alle 50ms
  if (currT - lastPosUpdate >= 50000) {
    updatePosition(pos[0] - lastPos[0], pos[1] - lastPos[1]);
    lastPos[0] = pos[0];
    lastPos[1] = pos[1];
    lastPosUpdate = currT;
  }

  if (!leftTriggered && true) { // && limit switch triggered
    target[1] = pos[1];
  }

  if (!rightTriggered && true) { // && limit switch triggered
    target[0] = pos[0];
  }

  isDriving = false;
  // loop through the motors
  for (int k = 0; k < NMOTORS; k++) {
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
    if(pwr == 0) {
      dir=0;
    }
    // signal the motor
    setMotor(dir, pwr, lpwm[k], rpwm[k]);
    // Check if the motor is driving
    // isDriving = isDriving || abs(abs(pos[k]) - abs(target[k])) <= 20;
    isDriving = isDriving || pwr > pwmCutoff;
  }



  if (!isDriving) {
    resetPosition();
  }
}
