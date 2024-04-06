#include <Arduino.h>
#include <util/atomic.h>

// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {2,18};
const int encb[] = {3,19};
const int lpwm[] = {9,11};
const int rpwm[] = {8,10};

bool freePath = true;

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
    } else if (command == 't') {
      String valueStr = input.substring(2); 
      float angle = valueStr.toFloat(); 
      turn(angle);
    }
  }
}

// void sendData() {
//   String data;
//   data += driving? "d" : "s";
//   data += "x";
//   data += String(x);
//   data += "y";
//   data += String(y);
//   data += "t";
//   data += String(theta);
//   data += " DEBUG ";
//   data += debug;
//   Serial.println(data);
// }

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


  for(int k = 0; k < NMOTORS; k++){
    // pinMode(enca[k],INPUT_PULLUP);
    // pinMode(encb[k],INPUT_PULLUP);
    // pinMode(lpwm[k],OUTPUT);
    // pinMode(lpwm[k],OUTPUT);

    pid[k].setParams(1,0,0,100);
  }
  
  // attachInterrupt((enca[0]),readEncoder<0>,RISING);
  // attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target[NMOTORS];
  target[0] = 7.639437 * 1000;
  target[1] = 7.639437 * 1200;

  // time difference
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
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,lpwm[k], rpwm[k]);
  }

  for (int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(posi[k]);
    Serial.print(" ");
  }
  Serial.println();
}
