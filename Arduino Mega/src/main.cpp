#include <Arduino.h>
#include "./main.h"
// Define pins

String debug = "";

float currentPwmLeft;
float currentPwmRight;

bool driving = false;

float x = 0;
float y = 0;
float theta = 0;
long int lastEncLeft=0; // maybe not needed
long int lastEncRight=0;

volatile long int encoderLeft=0;
volatile long int encoderRight=0;

int counter=0;
unsigned long previousMillis = 0;
const long updatePosInterval = 1000;

long int oldEncoderLeft;
long int oldEncoderRight;

long int leftEncoderChange;
long int rightEncoderChange;

float leftEnc1 = 0;
float rightEnc1 = 0;
float leftEnc2 = 0;
float rightEnc2 = 0;


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


long int getEncoderLeft() {
  return encoderLeft;
}

long int getEncoderRight() {
  return encoderRight;
}

void stopMotor() {setPwmValues(0, 0); }

void setEncoderZero() {encoderLeft=0; encoderRight=0;}

float getAngle(float input = theta) {
    float result = theta*180/M_PI;
    // result = fmod((result + 360.0), 360.0);
    // now in updatepos with theta
    return result;
}

void setPwmValues(float pwmLeft, float pwmRight) {
  float pL = abs(pwmLeft);
  float pR = abs(pwmRight);
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
  driving = !(pwmLeft == 0 && pwmRight == 0);
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
    sendData();
}

void updatePositionThread() { // NEED MILLIS
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= 100) {
      previousMillis = currentMillis;


      leftEnc2 = getEncoderLeft();
      rightEnc2 = getEncoderRight();
      updatePosition(leftEnc2 - leftEnc1, rightEnc2 - rightEnc1);
      leftEnc1 = getEncoderLeft();
      rightEnc1 = getEncoderRight();
    }
}



void turn(float degrees) {
    float distance = turnValue * degrees; // in mm
    float pulsesLeft = -1.0f * (distance * pulsesPerMM); // links rückwärts... sollte passen ig
    float pulsesRight = distance * pulsesPerMM;

    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    int lastEncLeft = getEncoderLeft();
    int lastEncRight = getEncoderRight();
    long int currentEncoderLeft = 0;
    long int currentEncoderRight = 0;
    long int currentPIDleft = 0;
    long int currentPIDright = 0;

    drive(-pwmSpeed, pwmSpeed);
    counter = 0;
    while((abs(currentEncoderLeft)+abs(currentEncoderRight))/2 < (abs(pulsesLeft)+abs(pulsesRight))/2) { // solange wir noch nicht da sind
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        currentPIDleft = getEncoderLeft() - lastEncLeft;
        currentPIDright = getEncoderRight() - lastEncRight;
        // check ob gegner auf stregge brauchen wir hier nicht
        counter++;
        // debug = "PULSES NEEDED IN TIME: " + String(pulsesPerSec/(1000/syncCounterTurn));
        if(counter >= syncCounterTurn) {
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) { // fehler vermeiden
                float newPwmLeft = (pulsesPerSec/(1000/syncCounterTurn) / abs(currentPIDleft) * currentPwmLeft); // geteilt durch 5 wegen syncCounterTurn
                float newPwmRight = (pulsesPerSec/(1000/syncCounterTurn) / abs(currentPIDright) * currentPwmRight);
                debug = String(newPwmLeft) +  " " + String(newPwmRight) + " , " + String(currentPIDleft) + " " + String(currentPIDright);
                drive(newPwmLeft, newPwmRight);
                lastEncLeft = getEncoderLeft();
                lastEncRight = getEncoderRight();
                // odom calc start
                // updatePosition(currentPIDleft, currentPIDright);
                // odom calc end
                getData();
                updatePositionThread();
            }
            counter = 0;
            getData();
            updatePositionThread();
        }
        delay(1);
        updatePositionThread();
        getData();
    }
    drive(0, 0);
    // updatePosition(currentPIDleft, currentPIDright);
    // odom manual start -> not recommended
    // theta += degrees;
    // theta = fmod((theta + 360.0), 360.0);
    // odom manual end
}

void driveDistance(int distance) {
    // RUN BEFORE DRIVING!!
    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    int lastEncLeft = getEncoderLeft();
    int lastEncRight = getEncoderRight();

    float distancePulses = distance * pulsesPerMM;

    // need these 2 lines to recalculate current enc values. 
    long int currentEncoderLeft = 0; // for driving
    long int currentEncoderRight = 0;
    long int currentPIDleft = 0;
    long int currentPIDright = 0;

    drive(pwmSpeed, pwmSpeed); // start with 100 pwm
    counter = 0;
    while(distancePulses > (currentEncoderLeft + currentEncoderRight)/2) { // might need correction
        // solange wir noch nicht da sind
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        currentPIDleft = getEncoderLeft() - lastEncLeft;
        currentPIDright = getEncoderRight() - lastEncRight;
        // hier check ob gegner auf strecke
        counter++;
        if(counter >= syncCounter) { //wenn bestimmte zeit vergangen
            // neue pwm werte basierend auf encoder daten berechnen und positionsbestimmung
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) {
                float newPwmLeft = currentPwmLeft * currentPIDright / currentPIDleft * currentPIDright / currentPIDleft;
                float newPwmRight = currentPwmRight * currentPIDleft / currentPIDright * currentPIDleft / currentPIDright;
                debug += "new pwm left: " + String(newPwmLeft);
                debug += " new pwm right: " + String(newPwmRight); 
                // std::cout << "before drive func: " << pulsesPerSec / abs(currentEncoderLeft) * currentPwmLeft << ", " << pulsesPerSec << ", " << abs(currentEncoderLeft) << ", " << currentPwmLeft << std::endl;
                drive(newPwmLeft, newPwmRight);
                lastEncLeft = getEncoderLeft();
                lastEncRight = getEncoderRight();
                // updatePosition(currentPIDleft, currentPIDright);
                getData();
            }
            counter = 0;
            getData();
        }
        delay(5);
        updatePositionThread();
        getData();
    }
    drive(0, 0); // stop motor
    // updatePosition(currentPIDleft, currentPIDright);
}










void getData() { // get the data and run the actions
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    char command = input.charAt(0);
    if (command == 's') {
      stopMotor();
    } else if (command == 'd') {
      String valueStr = input.substring(2);
      int distance = valueStr.toInt();
      driveDistance(distance);
    } else if (command == 'w') {
      // leftTriggered = false;
      // rightTriggered = false;
      driveDistance(3000);
    } else if (command == 't') {
      String valueStr = input.substring(2);
      float angle = valueStr.toFloat();
      turn(angle);
    } else if (command == 'g') {
      // String valueStr = input.substring(2);
      // int speed = valueStr.toInt();
      // currentPwm = speed < pwmMax ? speed : pwmMax;
    }
  }
}

void sendData() {
  String data;
  data += driving? "d" : "s";
  data += "x";
  data += String(x);
  data += "y";
  data += String(y);
  data += "t";
  data += String(theta);
  data += " DEBUG ";
  data += debug;
  debug = "";
  Serial.println(data);
}

void setup() {
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

// Loop function

void loop() {
  updatePositionThread(); // NEEDS TO RUN AS OFTEN AS POSSIBLE
  getData(); // get all data and process stuff
  sendData(); 
}