#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x4c, 0x75, 0x25, 0x37, 0x19, 0xF4};

// Variable to store if sending data was successful
String success;

//Must match the receiver structure
typedef struct struct_message {
  char colour[32];
  int success;
  int sima_ID;
  int path;
} struct_message;

bool flag = false;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

Servo servoLeft;
Servo servoRight;

int startMillis;

void stop(){
  servoLeft.write(90);
  servoRight.write(90);
}

void checkButton(){
  int buttonState = digitalRead(D6);

  while(buttonState){
    stop();
    buttonState = digitalRead(D6);
    delay(10);
  }
}

// drive: 9, 180
// turnLeft: 0, 14
// turnRight: 180, 166
void drive(int duration, int speed_l, int speed_r, int startMillis){
  for (int i = 0; i < duration/200; i++)
  {
    while((startMillis + 20000) < millis()){
      stop();
    }

    checkButton();

    int last_range = readToF();
    while (last_range < 100 && last_range != -1)
    {
      stop();
      last_range = readToF();
    }

    servoLeft.write(speed_l);
    servoRight.write(speed_r);
    
    delay(200);
  }
  stop();
}

void turn90(char direction, int startMillis)
{
  if (direction == 'r')
  {
    drive(120, 9, 0, startMillis);
  }
  else
  {
    drive(120, 14, 180, startMillis);
  }
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("colour: ");
  Serial.println(myData.colour);
  Serial.print("Success: ");
  Serial.println(myData.success);
  Serial.print("sima ID: ");
  Serial.println(myData.sima_ID);
  
  flag = true;
}