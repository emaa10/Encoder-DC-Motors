#include <Servo.h>


Servo servoLeft;
Servo servoRight;

int startMillis;

//define offsets
bool gegi;
int pwmOffset;
int turnOffsetL;
int turnOffsetR;

void stop(){
  servoLeft.write(90);
  servoRight.write(90);
}

void IRAM_ATTR emergencyStop(){
  if(digitalRead(D6) == LOW){
    servoLeft.write(90);
    servoRight.write(90);  

    servoLeft.detach();
    servoRight.detach();

    while(true){
      ESP.deepSleep(500e6);
      yield();
    }
  }
}

void accelerate(int steps){
  for(int i = 0; i < steps; i++){
    int last_range = readToF();

    while(last_range < 100 && last_range != -1 && gegi){
      servoLeft.write(90);
      servoRight.write(90);

      last_range = readToF();
    }

    servoLeft.write(90+i+pwmOffset);
    servoRight.write(90-i);
     
    delay(10);
  }
}


void drive(int duration){
  accelerate(40);

  for (int i = 0; i < (duration-1000)/100; i++) {
    int last_range = readToF();
    while (last_range < 100 && last_range != -1 && gegi)
    {
      servoLeft.write(90);
      servoRight.write(90);
      last_range = readToF();
    }

    servoLeft.write(130+pwmOffset);
    servoRight.write(50);

    delay(50);
  }
  stop();
  delay(200);
}

void driveFast(int duration){
  accelerate(40);

  for (int i = 0; i < (duration-1000)/100; i++) {
    int last_range = readToF();
    while (last_range < 100 && last_range != -1 && gegi)
    {
      servoLeft.write(90);
      servoRight.write(90);
      last_range = readToF();
    }

    servoLeft.write(161+pwmOffset);
    servoRight.write(20);

    delay(50);
  }
  stop();
  delay(200);
}

void turn90(char direction)
{
  if (direction == 'r')
  {
    servoLeft.write(130);
    servoRight.write(90);
    delay(325+turnOffsetR);
    stop();
  }
  else
  {
    servoLeft.write(90);
    servoRight.write(50);
    delay(325+turnOffsetL);
    stop();
  }
  delay(200);
}

void curve(int offset, int duration){
  accelerate(25);

  for (int i = 0; i < (duration-1000)/100; i++) {
    int last_range = readToF();
    while (last_range < 100 && last_range != -1)
    {
      servoLeft.write(90);
      servoRight.write(90);
      last_range = readToF();
    }

    servoLeft.write(130+pwmOffset+offset);
    servoRight.write(50);

    delay(50);
  }
  stop();
  delay(200);
}