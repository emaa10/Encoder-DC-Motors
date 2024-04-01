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
// odometry
float leftEnc1 = 0;
float rightEnc1 = 0;
float leftEnc2 = 0;
float rightEnc2 = 0;

int counter=0;
unsigned long previousMillis = 0;
const long updatePosInterval = 1000;

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

  print("pL: ");
  print(pL);
  print(" pR: ");
  print(pR);
  print(" pwmLeft: ");
  print(pwmLeft);
  print(" pwmRight: ");
  println(pwmRight);
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
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Theta: ");
    Serial.println(theta);
}

void updatePositionThread() { // NEED MILLIS
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= updatePosInterval) {
      previousMillis = currentMillis;
      // print("current");
      // print(currentMillis);
      // print(" previous ");
      // print(previousMillis);

      leftEnc2 = getEncoderLeft();
      rightEnc2 = getEncoderRight();
      updatePosition(leftEnc2 - leftEnc1, rightEnc2 - rightEnc1);
      leftEnc1 = getEncoderLeft();
      rightEnc1 = getEncoderRight();
    }
}

void turn(float degrees) {
    float distance = turnValue * degrees;
    float pulsesLeft = -1.0f * (distance * pulsesPerMM); // links rückwärts... sollte passen ig
    float pulsesRight = distance * pulsesPerMM;
    print("left pulses: ");
    print(pulsesLeft);
    print(" right pulses: ");
    println(pulsesRight);

    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    int lastEncLeft = getEncoderLeft();
    int lastEncRight = getEncoderRight();
    long int currentEncoderLeft = 0;
    long int currentEncoderRight = 0;
    long int currentPIDleft = 0;
    long int currentPIDright = 0;

    print(-pwmSpeed);
    print(" ");
    println(pwmSpeed);
    drive(-pwmSpeed, pwmSpeed); // links rückwrts
    counter = 0;
    while(currentEncoderLeft > pulsesLeft || currentEncoderRight < pulsesRight) { // solange wir noch nicht da sind
        print("enc left: ");
        print(currentEncoderLeft);
        print(" enc right: ");
        println(currentEncoderRight);
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        currentPIDleft = getEncoderLeft() - lastEncLeft;
        currentPIDright = getEncoderRight() - lastEncRight;
        // check ob gegner auf stregge brauchen wir hier nicht
        counter++;
        if(counter >= syncCounterTurn) {
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) { // fehler vermeiden
                float newPwmLeft = pulsesPerSec/(1000/syncCounterTurn) / abs(currentPIDleft) * currentPwmLeft; // geteilt durch 5 wegen syncCounterTurn
                float newPwmRight = pulsesPerSec/(1000/syncCounterTurn) / abs(currentPIDright) * currentPwmRight;
                // drive(newPwmLeft, newPwmRight);
                lastEncLeft = getEncoderLeft();
                lastEncRight = getEncoderRight();
                // odom calc start
                // updatePosition(currentPIDleft, currentPIDright);
                // odom calc end
            }
        }
        delay(5);
        updatePositionThread();
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
    print("alles davor: ");
    println(getEncoderLeft());
    float distancePulses = distance * pulsesPerMM;

    // need these 2 lines to recalculate current enc values. 
    long int currentEncoderLeft = 0; // for driving
    long int currentEncoderRight = 0;
    long int currentPIDleft = 0;
    long int currentPIDright = 0;

    drive(pwmSpeed, pwmSpeed); // start with 100 pwm
    counter = 0;
    while(distancePulses > (currentEncoderLeft + currentEncoderRight)/2) { // might need correction
        print("durchschnitt enc: ");
        println((currentEncoderLeft + currentEncoderRight)/2);
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
                float newPwmLeft = pulsesPerSec / abs(currentPIDleft) * currentPwmLeft;
                print("currentPIDleft: ");
                print(currentPIDleft);
                print(" currentpwmleft: ");
                println(currentPwmLeft);
                float newPwmRight = pulsesPerSec / abs(currentPIDright) * currentPwmRight;
                // std::cout << "before drive func: " << pulsesPerSec / abs(currentEncoderLeft) * currentPwmLeft << ", " << pulsesPerSec << ", " << abs(currentEncoderLeft) << ", " << currentPwmLeft << std::endl;
                drive(newPwmLeft, newPwmRight);
                print("Newpwmleft: ");
                print(newPwmLeft);
                print(", Newpwmright: ");
                println(newPwmRight);
                lastEncLeft = getEncoderLeft();
                lastEncRight = getEncoderRight();
                // updatePosition(currentPIDleft, currentPIDright);
            }
            counter = 0;
        }
        delay(5);
        updatePositionThread();
    }
    drive(0, 0); // stop motor
    // updatePosition(currentPIDleft, currentPIDright);
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

  // while(pullCordConnected()) {delay(5); } // solange pullcord connected is
  // delay(500);
  // setEncoderZero();
  // delay(1000);

  println("START");

  // driveDistance(1000);
  turn(90);
  // drive(50, 0);
  // delay(10000);
  // drive(0,0);


  println("SIND DA");
}

void loop()
{
  updatePositionThread(); // NEEDS TO RUN AS OFTEN AS POSSIBLE
  // setPwmValues(50,50);
  // analogWrite(LEFT_LPWM, 100);
  // analogWrite(RIGHT_RPWM, 100);
  print("Encoder left: ");
  print(getEncoderLeft());
  print(", Encoder right: ");
  println(getEncoderRight());
  delay(5);
}