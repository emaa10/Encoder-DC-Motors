#define spinnerStepperDIR 14
#define spinnerStepperSTEP 12

#define beltStepperDIR 32
#define beltStepperSTEP 33

#define limitSwitch 26

#define beltLimit 705
#define beltDroppingPos 440
#define beltMiddlePos 200


int beltPos = beltLimit;
int beltState = 0;
int spinnerState = 0;

void initialiseStepper(){
  pinMode(spinnerStepperDIR, OUTPUT); // Step
  pinMode(spinnerStepperSTEP, OUTPUT); // Richtung

  pinMode(beltStepperDIR, OUTPUT);
  pinMode(beltStepperSTEP, OUTPUT);
}

void spinnerTurnLeft() {
  digitalWrite(spinnerStepperDIR, HIGH);
  digitalWrite(spinnerStepperSTEP,HIGH);
  delayMicroseconds(750);
  digitalWrite(spinnerStepperSTEP,LOW);
  delayMicroseconds(750);
}

void spinnerTurnRight() {
  digitalWrite(spinnerStepperDIR, LOW);
  digitalWrite(spinnerStepperSTEP,HIGH);
  delayMicroseconds(750);
  digitalWrite(spinnerStepperSTEP,LOW);
  delayMicroseconds(750);
}

void beltUp() {
  if(beltPos > 0){
    digitalWrite(beltStepperDIR, HIGH);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(750);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(750);
    beltPos --;
  }
}

void beltDown() {
  if(beltPos < beltLimit) {
    digitalWrite(beltStepperDIR, LOW);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(750);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(750);
    beltPos++;
  }
}

void beltDropping() {
  if(beltPos > beltDroppingPos) {
    digitalWrite(beltStepperDIR, HIGH);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(750);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(750);
    beltPos --;    
  } else if(beltPos < beltDroppingPos) {
    digitalWrite(beltStepperDIR, LOW);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(750);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(750);
    beltPos++;
  } else if(beltPos == beltDroppingPos) {
    beltState = 0;
  }
}

void beltMiddle() {
  if(beltPos > beltMiddlePos) {
    digitalWrite(beltStepperDIR, HIGH);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(750);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(750);
    beltPos --;    
  } else if(beltPos < beltMiddlePos) {
    digitalWrite(beltStepperDIR, LOW);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(750);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(750);
    beltPos++;
  } else if(beltPos == beltMiddlePos) {
    beltState = 0;
  }
}