#include "espnow_handler.h"
#include <esp_now.h>
#include <WiFi.h>

static EspNowCallback userCb = nullptr;
static struct_message incomingData;

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingBytes, int len) {
  Serial.println("\n[ESP-NOW] Packet received");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.printf("From MAC: %s len=%d\n", macStr, len);

  if (len != sizeof(incomingData)) {
    Serial.println("[ESP-NOW] Wrong size.");
    return;
  }
  memcpy(&incomingData, incomingBytes, sizeof(incomingData));
  if (userCb) userCb(incomingData);
}

void espnowInit(EspNowCallback cb) {
  userCb = cb;

  WiFi.mode(WIFI_STA);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed!");
  } else {
    Serial.println("[ESP-NOW] initialized");
    esp_now_register_recv_cb(onDataRecv);
  }
}
