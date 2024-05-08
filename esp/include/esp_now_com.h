#include <esp_now.h>
#include <WiFi.h>

bool gegi = true;
bool teamBlue = false;

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS

uint8_t sima1Address[] = {0x4C, 0x75, 0x25, 0x37, 0x19, 0xF4};
uint8_t sima2Address[] = {0xC8, 0xC9, 0xA3, 0x1A, 0xB9, 0x87};
uint8_t sima3Address[] = {0xC8, 0xC9, 0xA3, 0x1B, 0x0A, 0xE3};
uint8_t sima4Address[] = {0xC8, 0xC9, 0xA3, 0x1A, 0xE5, 0x77};


typedef struct struct_message
{
  bool gegi;
  bool colourBlue;
  int path;
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
}

void sendSimas(){
  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;

  esp_now_send(sima1Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi
  ;
  message.colourBlue = teamBlue;
  message.path = 11;

  esp_now_send(sima2Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;

  esp_now_send(sima3Address, (uint8_t *) &message, sizeof(message));
  delay(100);

  message.gegi = gegi;
  message.colourBlue = teamBlue;
  message.path = 11;

  esp_now_send(sima4Address, (uint8_t *) &message, sizeof(message));
  delay(100);
}