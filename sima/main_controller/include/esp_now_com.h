#include <esp_now.h>
#include <WiFi.h>

bool gegi = true;
bool teamBlue = false;

//master esp32 address:E8:31:CD:E6:AD:0C
uint8_t sima_Adresses[6][6] = {
  {0x08, 0x3A, 0x8D, 0xCF, 0xAF, 0xBA},
  {0xC8, 0xC9, 0xA3, 0x1B, 0x0E, 0x23},
  {0xC8, 0xC9, 0xA3, 0x1B, 0x0A, 0xE3},
  {0xC8, 0xC9, 0xA3, 0x1A, 0xB9, 0x87},
  {0xC8, 0xC9, 0xA3, 0x1A, 0xDF, 0x19},
  {0x4C, 0x75, 0x25, 0x37, 0x19, 0xF4}
};
String success;

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


esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void initialiseESP_NOW(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  for(int i = 0; i < 6; i++){
    // Register peers
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, sima_Adresses[i], 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
  }
}

void sendSimas(){
  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 1;
  message.pwmOffset = -8;
  message.turnOffsetL = -30;
  message.turnOffsetR = -72;

  esp_now_send(sima_Adresses[0], (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 3;
  message.pwmOffset = -4;
  message.turnOffsetL = 40;
  message.turnOffsetR = 0;

  esp_now_send(sima_Adresses[1], (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 2;
  message.pwmOffset = -3;
  message.turnOffsetL = -25;
  message.turnOffsetR = 0;

  esp_now_send(sima_Adresses[2], (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;
  message.pwmOffset = 0;
  message.turnOffsetL = 245;
  message.turnOffsetR = 225;

  esp_now_send(sima_Adresses[3], (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 4;
  message.pwmOffset = -1;
  message.turnOffsetL = 200;
  message.turnOffsetR = 135;

  esp_now_send(sima_Adresses[4], (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;
  message.pwmOffset = -1;
  message.turnOffsetL = 190;
  message.turnOffsetR = 145;

  esp_now_send(sima_Adresses[5], (uint8_t *) &message, sizeof(message));
}
