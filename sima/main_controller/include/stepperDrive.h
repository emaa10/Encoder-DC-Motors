#define spinnerStepperDIR 14
#define spinnerStepperSTEP 12

#define beltStepperDIR 32
#define beltStepperSTEP 33

#define limitSwitch 26

#define beltLimit 705
#define beltMiddlePos 425

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
  delayMicroseconds(500);
  digitalWrite(spinnerStepperSTEP,LOW);
  delayMicroseconds(500);
}

void spinnerTurnRight() {
  digitalWrite(spinnerStepperDIR, LOW);
  digitalWrite(spinnerStepperSTEP,HIGH);
  delayMicroseconds(500);
  digitalWrite(spinnerStepperSTEP,LOW);
  delayMicroseconds(500);
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