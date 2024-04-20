#include <esp_now.h>
#include <WiFi.h>

bool gegi = true;
bool teamBlue = false;

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t sima1Address[] = {0x08, 0x3A, 0x8D, 0xCF, 0xAF, 0xBA};
uint8_t sima2Address[] = {0xC8, 0xC9, 0xA3, 0x1B, 0x0E, 0x23};
uint8_t sima3Address[] = {0xC8, 0xC9, 0xA3, 0x1B, 0x0A, 0xE3};
uint8_t sima4Address[] = {0xC8, 0xC9, 0xA3, 0x1A, 0xB9, 0x87};
uint8_t sima5Address[] = {0xC8, 0xC9, 0xA3, 0x1A, 0xDF, 0x19};
uint8_t sima6Address[] = {0x4C, 0x75, 0x25, 0x37, 0x19, 0xF4};
  

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

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void initSimas() { 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, sima1Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, sima2Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, sima3Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  /// register fourth peer
  memcpy(peerInfo.peer_addr, sima4Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  /// register fifth peer
  memcpy(peerInfo.peer_addr, sima5Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  /// register sixth peer
  memcpy(peerInfo.peer_addr, sima6Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void sendSimas(){
  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 1;
  message.pwmOffset = teamBlue? -5 : -12;
  message.turnOffsetL = 0;
  message.turnOffsetR = -100;

  esp_now_send(sima1Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 2;
  message.pwmOffset = teamBlue? 7 : -3;
  message.turnOffsetL = 30;
  message.turnOffsetR = 0;

  esp_now_send(sima2Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 3;
  message.pwmOffset = teamBlue? -3 : -3;
  message.turnOffsetL = -50;
  message.turnOffsetR = -25;

  esp_now_send(sima3Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;
  message.pwmOffset = teamBlue? 0 : 0;
  message.turnOffsetL = 245;
  message.turnOffsetR = 225;

  esp_now_send(sima4Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 2;
  message.pwmOffset = teamBlue? -1 : -1;
  message.turnOffsetL = 200;
  message.turnOffsetR = 135;

  esp_now_send(sima5Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;
  message.pwmOffset = teamBlue? -1 : -1;
  message.turnOffsetL = 290;
  message.turnOffsetR = 145;

  esp_now_send(sima6Address, (uint8_t *) &message, sizeof(message));
}