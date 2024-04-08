#include <Arduino.h>
#include <util/atomic.h>
// #include "./main.h"
const float pulsesPerEncRev = 1200;
const float encWheelDiameterCM = 5;
const float motorWheelDiameterCM = 7;
const float encWheelScope = encWheelDiameterCM * M_PI; 
const float motorWheelScope = motorWheelDiameterCM * M_PI; // distance travelled per rev
const float pulsesPerRev = pulsesPerEncRev * (motorWheelScope / encWheelScope);
const float pulsesPerMM = pulsesPerRev / motorWheelScope / 10;
const float pulsesPerCM = pulsesPerRev / motorWheelScope;
const float pwmSpeed = 100; //default pwm speed
const float pulsesPerSec = pulsesPerRev; //goal pulses per sec 1680, 1 round per second
const float wheelDistance = 128; //abstand der encoderräder in mm, muss vllt geändert werden
const float wheelDistanceBig = 204; // in mm, muss vllt geändert werden
const float turnValue = wheelDistance * M_PI / 360; // abstand beider räder um 1° zu fahren


// How many motors
const int NMOTORS = 2;

int target[NMOTORS] = {0};
// target[0] = 0;
// target[1] = 0;

// Pins
const int enca[] = {2,18};
const int encb[] = {3,19};
const int lpwm[] = {9,11};
const int rpwm[] = {8,10};

bool freePath = true;
bool reachedGoal = false;
float x = 225;
float y = 225;
float theta = 0;
float leftEncoderChange=0;
float rightEncoderChange=0;

//Class
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(100), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

// Globals
long prevT = 0;
volatile int posi[] = {0,0};
int posP[] = {0, 0};

// PID class instances
SimplePID pid[NMOTORS];

void setMotor(int dir, int pwmVal, int lpwm, int rpwm){
  if(dir == 1){
    analogWrite(lpwm,pwmVal);
  }
  else if(dir == -1){
    analogWrite(rpwm,pwmVal);
  }
  else{
    analogWrite(lpwm,0);
    analogWrite(rpwm,0);
  }  
}

#define LEFT_ENC_A_PHASE 18
#define LEFT_ENC_B_PHASE 19
#define RIGHT_ENC_A_PHASE 2
#define RIGHT_ENC_B_PHASE 3

void ai0() {
  if (digitalRead(LEFT_ENC_A_PHASE) == LOW) { posi[1]++; }
  else { posi[1]--; }
}

void ai1() {
  if (digitalRead(LEFT_ENC_B_PHASE) == LOW) { posi[1]--; }
  else { posi[1]++; }
}

void bi0() {
  if (digitalRead(RIGHT_ENC_A_PHASE) == LOW) { posi[0]++; }
  else { posi[0]--; }
}

void bi1() {
  if (digitalRead(RIGHT_ENC_B_PHASE) == LOW) { posi[0]--; }
  else { posi[0]++; }
}

void sendData() {
  String data;
  data += reachedGoal? "s" : "d";
  data += "x";
  data += String(x);
  data += "y";
  data += String(y);
  data += "t";
  data += String(theta);
  Serial.println(data);
}

void updatePosition(float leftEncChange, float rightEncChange) {
  // posi[0] = 0;
  // posi[1] = 0;
  // target[0] = 0;
  // target[1] = 0;

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

void drive(){
  Serial.print("posi 0: ");
  Serial.println(posi[0]);
  Serial.print("posi 1: ");
  Serial.println(posi[1]);
  Serial.print("target 0: ");
  Serial.println(target[0]);
  Serial.print("target 1: ");
  Serial.println(target[1]);
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  leftEncoderChange = pos[0] - posP[0];
  rightEncoderChange = pos[1] - posP[1];
  posP[0] = pos[0];
  posP[1] = pos[1];
  updatePosition(leftEncoderChange, rightEncoderChange);

  if (freePath) {
    // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);

      if(pwr < 15){
        pwr = 15;
      }
      // signal the motor
      setMotor(dir,pwr,lpwm[k], rpwm[k]);
    }

    reachedGoal = true; // needs testing
  }

  if(abs(pos[0] - target[0]) < 8 && abs(pos[1] - target[1]) < 8) {
    reachedGoal = true;
    Serial.println(pos[0]);
    Serial.println(target[0]);
  } else {
    reachedGoal = false;
  }
}

void driveUntilSwitch() {
  posi[0] = 0;
  posi[1] = 0;

  target[0] = 10000000000; // random high value
  target[1] = 10000000000; // random high value

  reachedGoal = false;

  while(bool LIMITSWTRIGGERED=1){ // HIER ODER STATEMENT FÜR LIMIT SW
    // getData();
    drive();
  }

  reachedGoal = true;
  // HIER SET POS
  // x = ;
  // y = ;

  setMotor(0,0,lpwm[0], rpwm[0]);
  setMotor(0,0,lpwm[1], rpwm[1]);

  delay(250);
}

void driveDistance(int distance) {
  posi[0] = 0;
  posi[1] = 0;

  target[0] = 7.639437 * distance;
  target[1] = 7.639437 * distance;

  reachedGoal = false;

  while(!reachedGoal){
    // getData();
    drive();
  }

  setMotor(0,0,lpwm[0], rpwm[0]);
  setMotor(0,0,lpwm[1], rpwm[1]);

  delay(250);
}

void turnAngle(int degree) {
  posi[0] = 0;
  posi[1] = 0;

  int distance = 128*3.1415926/360*degree;
  target[0] = 7.639437 * distance;
  target[1] = -7.639437 * distance;

  reachedGoal = false;

  while(!reachedGoal){
    // getData();
    drive();
  }

  setMotor(0,0,lpwm[0], rpwm[0]);
  setMotor(0,0,lpwm[1], rpwm[1]);

  delay(250);
  sendData();
}

void getData() { // get the data and run the actions
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 
    char command = input.charAt(0);
  
    if (command == 's') {
      freePath = false;
    } else if (command == 'd') {
      String valueStr = input.substring(2); 
      int distance = valueStr.toInt();
      driveDistance(distance); 
    } else if (command == 'w') {
      driveUntilSwitch();
    } else if (command == 't') {
      String valueStr = input.substring(2); 
      float angle = valueStr.toFloat(); 
      turnAngle(angle);
    }
  }
}



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


  for(int k = 0; k < NMOTORS; k++){
    pid[k].setParams(1,0,0,100);
  }
}



void loop() {  
  getData();
}