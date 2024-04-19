#include <Arduino.h>

#define StepperDIR 14
#define StepperSTEP 12

#define beltStepperDIR 36
#define beltStepperSTEP 39

#define limitSwitch 35

void initialiseStepper(){
    pinMode(StepperDIR, OUTPUT); // Step
    pinMode(StepperSTEP, OUTPUT); // Richtung
}

void stepperTurnLeft() {
    digitalWrite(StepperDIR, HIGH);
    digitalWrite(12,HIGH);
    delayMicroseconds(500);
    digitalWrite(12,LOW);
    delayMicroseconds(500);
}

void stepperTurnRight() {
    digitalWrite(StepperDIR, LOW);
    digitalWrite(12,HIGH);
    delayMicroseconds(500);
    digitalWrite(12,LOW);
    delayMicroseconds(500);
}

void initialiseBeltStepper(){
    pinMode(beltStepperDIR, OUTPUT); // Step
    pinMode(beltStepperSTEP, OUTPUT); // Richtung
}

void beltStepperTurnLeft() {
    digitalWrite(beltStepperDIR, HIGH);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(500);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(500);
}

void beltStepperTurnRight() {
    digitalWrite(beltStepperDIR, LOW);
    digitalWrite(beltStepperSTEP,HIGH);
    delayMicroseconds(500);
    digitalWrite(beltStepperSTEP,LOW);
    delayMicroseconds(500);
}