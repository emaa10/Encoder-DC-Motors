#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "sensor.h"
#include "drive.h"
#include "paths.h"

// REPLACE WITH THE MAC Address of your receiver
uint8_t main_controller_address[] = {0xE8, 0x31, 0xCD, 0xE6, 0xAD, 0x0C};

bool flag = false;


// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  bool gegi;
  bool colourBlue;
  int path;
  int pwmOffset;
  int turnOffsetL;
  int turnOffsetR;
} struct_message;

struct_message message;


// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  memcpy(&message, incomingData, sizeof(message));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Path:  ");
  Serial.println(message.path);
  Serial.print("Colour:  ");
  Serial.println(message.colourBlue);

  gegi = message.gegi;
  colorBlue = message.colourBlue;
  path = message.path;
  pwmOffset = message.pwmOffset;
  turnOffsetL = message.turnOffsetL;
  turnOffsetR = message.turnOffsetR;

  flag = true;
}

void setup() {
  // Init Serial Monitor
  Serial.begin(9600);

  servoLeft.attach(D5);
  servoRight.attach(D7);

  pinMode(D6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(D6), emergencyStop, FALLING);

  while (! Serial)
      delay(1);
  
   if (!lox.begin()){
      Serial.println(F("Failed to boot VL53L0X"));
  while(1);
  }
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  delay(500);
}
 
void loop() {  
  // readToF();
  if(flag){
    drivePath();

    servoLeft.detach();
    servoRight.detach();

    while(true) delay(500);
  }
}