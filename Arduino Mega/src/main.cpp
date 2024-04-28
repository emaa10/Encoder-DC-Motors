#include "simplepid.h"
#include <Arduino.h>
#include <util/atomic.h>

// Define pins

#define LEFT_ENC_A_PHASE 2
#define LEFT_ENC_B_PHASE 3
#define RIGHT_ENC_A_PHASE 18
#define RIGHT_ENC_B_PHASE 19

const int lpwm[] = {8, 11};
const int rpwm[] = {9, 10};
String DEBUG = "";

// Define Globals

#define NMOTORS 2
#define pwmCutoff 20 // Set minimum drivable pwm value
#define pulsesCutoff 6
#define pwmMax 200
int currentPwm = 200;
int lastpwm = 0;
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
    127; // abstand der encoderräder in mm, muss vllt geändert werden
const float wheelDistanceBig = 204; // in mm, muss vllt geändert werden
// const float turnValue =
//     wheelDistance * M_PI / 360; // abstand beider räder um 1° zu fahren

const float pulsesValue = pulsesPerMM;

float x = 225;
float y = 225;
float theta = 0;

bool teamYellow = false;

float extrax = 0;
float extray = 0;
float extraTheta = 0;

bool isDriving = false;
bool stopped = false;

long lastPosUpdate;
bool limitSwitchDrive;

// Encoder read functions

void ai0() {
  if (digitalRead(LEFT_ENC_A_PHASE) == LOW) {
    posi[1]--;
  } else {
    posi[1]++;
  }
}

void ai1() {
  if (digitalRead(LEFT_ENC_B_PHASE) == LOW) {
    posi[1]++;
  } else {
    posi[1]--;
  }
}

void bi0() {
  if (digitalRead(RIGHT_ENC_A_PHASE) == LOW) {
    posi[0]--;
  } else {
    posi[0]++;
  }
}

void bi1() {
  if (digitalRead(RIGHT_ENC_B_PHASE) == LOW) {
    posi[0]++;
  } else {
    posi[0]--;
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

void updatePosition();
void resetPosition() {
  updatePosition();

  x += extrax;
  y += extray;
  extrax = 0;
  extray = 0;
  theta += extraTheta;
  while (theta > 2 * M_PI) {
    theta -= 2 * M_PI;
  }
  while (theta < -2 * M_PI) {
    theta += 2 * M_PI;
  }
  extraTheta = 0;

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

  target[0] = pulsesValue * distance;
  target[1] = pulsesValue * distance;
}

void turnAngle(int degree) {
  resetPosition();

  int distance = wheelDistance * M_PI / 360 * degree;
  target[0] = -pulsesValue * distance;
  target[1] = pulsesValue * distance;
}

// Serial Communication

void getData() { // get the data and run the actions
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    char command = input.charAt(0);
    if (command == 's') {
      stopped = true;
    } else if (command == 'c') {
      stopped = false;
    } else if (command == 'd') {
      String valueStr = input.substring(2);
      int distance = valueStr.toInt();
      driveDistance(distance);
    } else if (command == 'w') {
      String valueStr = input.substring(2);
      bool dir = valueStr.toInt();
      driveDistance(dir ? 300 : -300);
      limitSwitchDrive = true;
    } else if (command == 't') {
      String valueStr = input.substring(2);
      float angle = valueStr.toFloat();
      turnAngle(angle);
    } else if (command == 'g') {
      String valueStr = input.substring(2);
      int speed = valueStr.toInt();
      currentPwm = speed < pwmMax ? speed : pwmMax;
    } else if (command == 't') { // set team
      String valueStr = input.substring(2);
      int value = valueStr.toInt();
      teamYellow = (value = 1) ? true : false;
    }
  }
}

void sendData() {
  String data;
  data += isDriving ? "d" : "s";
  data += "x";
  data += String(x + extrax);
  data += "y";
  data += String(y + extray);
  data += "t";
  data += String(theta + extraTheta);
  DEBUG += "Theta: " + String(theta * 180 / PI);
  DEBUG += " ExtraTheta: " + String(extraTheta * 180 / PI);
  data += " ";
  data += DEBUG;
  Serial.println(data);
}

// Position update function

void updatePosition() {
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NMOTORS; k++) {
      pos[k] = posi[k];
    }
  }

  float leftEncChange = pos[1] - lastPos[1];
  float rightEncChange = pos[0] - lastPos[0];
  lastPos[0] = pos[0];
  lastPos[1] = pos[1];

  float leftDistance = pos[1] / pulsesPerMM;
  float rightDistance = pos[0] / pulsesPerMM;
  float distance = (leftDistance + rightDistance) / 2;
  float dTheta = (leftDistance - rightDistance) / wheelDistance;
  extrax = distance * cos(theta + dTheta);
  extray = distance * sin(theta + dTheta);
  extraTheta = dTheta;
  // extraTheta = fmod((extraTheta + 2 * M_PI), (2 * M_PI)); // test in radian
  while (extraTheta > 2 * M_PI) {
    extraTheta -= 2 * M_PI;
  }
  while (extraTheta < -2 * M_PI) {
    extraTheta += 2 * M_PI;
  }

  int maxD = fabs(target[0] - pos[0]);
  maxD = maxD < fabs(target[1] - pos[1]) ? fabs(target[1] - pos[1]) : maxD;
  if ((fabs(leftEncChange) < pulsesCutoff &&
       fabs(rightEncChange) < pulsesCutoff && maxD < 20) ||
      (limitSwitchDrive && fabs(leftEncChange) < 10 &&
       fabs(rightEncChange) < 10 && fabs(pos[0]) > 10)) {
    isDriving = false;
    target[0] = pos[0];
    target[1] = pos[1];
    limitSwitchDrive = false;
  } else {
    isDriving = true;
  }

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
    pid[k].setParams(0.7, 0.2, 0.05, 100);
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
    updatePosition();
    lastPosUpdate = currT;
  }

  long pwm[NMOTORS];
  int dir[NMOTORS];
  float scaledFactor[NMOTORS];
  // loop through the motors

  lastpwm = lastpwm + 1;
  lastpwm = lastpwm > currentPwm ? currentPwm : lastpwm < pwmCutoff ? pwmCutoff : lastpwm;
  for (int k = 0; k < NMOTORS; k++) {
    // evaluate the control signal
    pid[k].evalu(pos[k], target[k], deltaT, pwm[k], dir[k]);
    // if (pwm[k] > currentPwm) {
    //   pwm[k] = currentPwm;
    // }
    scaledFactor[k] = (float)pwm[k] / lastpwm;
  }
  //  Serial.println("pwmleft: " + String(pwm[1]) + " pwmright: " +
  //  String(pwm[0]));
  float maxFactor = max(scaledFactor[0], scaledFactor[1]);
  if (maxFactor > 1) {
    pwm[0] /= maxFactor;
    pwm[1] /= maxFactor;
    // Serial.println("Pwm 0: " + String(pwm[0]) + " Pwm 1: " + String(pwm[1]));
  }

  for (int k = 0; k < NMOTORS; k++) {
    if (stopped) {
      setMotor(0, 0, lpwm[k], rpwm[k]);
    } else {
      setMotor(dir[k], pwm[k], lpwm[k], rpwm[k]);
    }
  }
  lastpwm = max(pwm[0], pwm[1]);
}
