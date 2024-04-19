#include "Arduino.h"
#include "sensor.h"
#include "com.h"
#include "paths.h"

void setup() {
  // Init Serial Monitor
  Serial.begin(9600);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  servoLeft.attach(D5);
  servoRight.attach(D7);
  pinMode(D6, INPUT_PULLUP);

  while (! Serial)
      delay(1);
  
   if (!lox.begin()){
      Serial.println(F("Failed to boot VL53L0X"));
  while(1);
  }
}
 
void loop() {  
  readToF();
  if(!digitalRead(D6)){
    servoLeft.write(130);
    servoRight.write(50);
    delay(5000);
    servoLeft.write(90);
    servoRight.write(90);
  } 
}