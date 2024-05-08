#define potterStepperDIR 14
#define potterStepperSTEP 12

#define slotterStepperDIR 32
#define slotterStepperSTEP 33

#define beltTopPos 705
#define beltDroppingPos 440
#define beltMiddlePos 300

int beltState;

int currentPosSlotter = 0;
int currentPosPotter = 0;

void initialiseStepper(){
  pinMode(potterStepperDIR, OUTPUT); // Step
  pinMode(potterStepperSTEP, OUTPUT); // Richtung

  pinMode(slotterStepperDIR, OUTPUT);
  pinMode(slotterStepperSTEP, OUTPUT);
}

void beltDrive(String input){
  if(input == "S_DOWN_P_DOWN"){
    digitalWrite(potterStepperDIR, LOW);
    digitalWrite(slotterStepperDIR, HIGH);

    while(currentPosPotter > 0 && currentPosSlotter > 0){
      digitalWrite(potterStepperSTEP,HIGH);
      digitalWrite(slotterStepperSTEP,HIGH);
      delayMicroseconds(750);
      digitalWrite(potterStepperSTEP,LOW);
      digitalWrite(slotterStepperSTEP,LOW);
      delayMicroseconds(750);
      currentPosPotter--;
      currentPosSlotter--;
    }
    while(currentPosSlotter > 0){
      digitalWrite(slotterStepperSTEP,HIGH);
      delayMicroseconds(750);
      digitalWrite(slotterStepperSTEP,LOW);
      delayMicroseconds(750);
      currentPosSlotter--;
    }
  } else if(input == "S_UP_P_DOWN"){ 
    digitalWrite(slotterStepperDIR, LOW);

    while(currentPosSlotter < beltTopPos){
      digitalWrite(slotterStepperSTEP,HIGH);
      delayMicroseconds(750);
      digitalWrite(slotterStepperSTEP,LOW);
      delayMicroseconds(750);
      currentPosSlotter++;
    }
  } else if(input == "S_UP_P_UP"){
    digitalWrite(potterStepperDIR, HIGH);
    digitalWrite(slotterStepperDIR, LOW);

    while(currentPosPotter < beltTopPos && currentPosSlotter < beltTopPos){
      digitalWrite(potterStepperSTEP,HIGH);
      digitalWrite(slotterStepperSTEP,HIGH);
      delayMicroseconds(750);
      digitalWrite(potterStepperSTEP,LOW);
      digitalWrite(slotterStepperSTEP,LOW);
      delayMicroseconds(750);
      currentPosPotter++;
      currentPosSlotter++;
    }
    while(currentPosPotter < beltTopPos){
      digitalWrite(potterStepperSTEP,HIGH);
      delayMicroseconds(750);
      digitalWrite(potterStepperSTEP,LOW);
      delayMicroseconds(750);
      currentPosPotter++;
    }
  } else if(input == "S_MID_P_MID"){
    digitalWrite(potterStepperDIR, HIGH);
    digitalWrite(slotterStepperDIR, LOW);
    if(currentPosPotter < beltTopPos){
      digitalWrite(potterStepperDIR, HIGH);
      digitalWrite(slotterStepperDIR, LOW);
      while(currentPosPotter < beltMiddlePos && currentPosSlotter < beltMiddlePos){
        digitalWrite(potterStepperSTEP,HIGH);
        digitalWrite(slotterStepperSTEP,HIGH);
        delayMicroseconds(750);
        digitalWrite(potterStepperSTEP,LOW);
        digitalWrite(slotterStepperSTEP,LOW);
        delayMicroseconds(750);
        currentPosPotter++;
        currentPosSlotter++;
      }
      while(currentPosPotter < beltMiddlePos){
        digitalWrite(potterStepperSTEP,HIGH);
        delayMicroseconds(750);
        digitalWrite(potterStepperSTEP,LOW);
        delayMicroseconds(750);
        currentPosPotter++;
      }
    } else if(currentPosPotter > beltMiddlePos){
      digitalWrite(potterStepperDIR, LOW);
      digitalWrite(slotterStepperDIR, HIGH);
      while(currentPosPotter > beltMiddlePos && currentPosSlotter > beltMiddlePos){
        digitalWrite(potterStepperSTEP,HIGH);
        digitalWrite(slotterStepperSTEP,HIGH);
        delayMicroseconds(750);
        digitalWrite(potterStepperSTEP,LOW);
        digitalWrite(slotterStepperSTEP,LOW);
        delayMicroseconds(750);
        currentPosPotter--;
        currentPosSlotter--;
      }
    }
  }
}
