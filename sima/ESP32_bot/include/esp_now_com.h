#include <esp_now.h>
#include <WiFi.h>

//master esp32 address:E8:31:CD:E6:AD:0C

uint8_t broadcastAddress[] = {0xC8,0xC9,0xA3,0x1A,0xE5,0x77};

String success;

typedef struct struct_message {
    long number;  
} struct_message;

struct_message message;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&message, incomingData, sizeof(message));
  Serial.print("Bytes received: ");
  Serial.println(len);

  esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
}

void initialiseESP_NOW(){
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}