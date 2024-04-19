#include "stepperDrive.h"
#include "display.h"
#include "esp_now_com.h"
#include "servos.h"
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(limitSwitch, INPUT_PULLUP);
  
  initialiseESP_NOW();
  initialiseDisplay();
  initialiseServos();
  initialiseStepper();
  initialiseBeltStepper();

}
 
void loop() {
  // displayInteger(random(0,9));
  // myservo.write(0);
  // myservo1.write(0);
  // myservo2.write(0);
  // myservo3.write(0);
  // myservo4.write(0);
  // myservo5.write(0);
  // delay(2000);

  // displayInteger(random(0,9999));
  // myservo.write(180);
  // myservo1.write(180);
  // myservo2.write(180);
  // myservo3.write(180);
  // myservo4.write(180);
  // myservo5.write(180);

  // delay(2000);

  // beltStepperTurnLeft();

  Serial.println(digitalRead(limitSwitch));
  delay(100);
}