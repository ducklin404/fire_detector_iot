#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>
#include "esp_wifi.h"

const int MQ5_PIN = 33;
const int FLAME_PIN = 35;
const int DHT_PIN = 23;
const int DEVICE_ID = 2;

uint8_t broadcastAddress[] = {0x1C, 0x69, 0x20, 0xB8, 0xC7, 0x44};

// Force both sender and gateway to same channel
#define ESPNOW_CHANNEL 6

typedef struct struct_message {
  int id;
  float gasAnalog;
  float flameAnalog;
  float temperature;
  float humidity;
} struct_message;

struct_message myData;

#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting ESP32 sensor sender...");

  WiFi.mode(WIFI_STA);
  delay(100);

  // Force channel before ESP-NOW init
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("ESP-NOW channel set to %d\n", ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized.");

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.print("Added peer: ");
    for (int i = 0; i < 6; i++) {
      if (i) Serial.print(":");
      Serial.print(broadcastAddress[i], HEX);
    }
    Serial.println();
  } else {
    Serial.println("Failed to add peer");
  }

  dht.begin();
  analogSetPinAttenuation(MQ5_PIN, ADC_11db);
  analogSetPinAttenuation(FLAME_PIN, ADC_11db);

  Serial.println("Give sensors a few seconds to stabilize...");
  delay(2000);
}

void loop() {
  int mqRaw = analogRead(MQ5_PIN);
  int flameRaw = analogRead(FLAME_PIN);

  float mqValue = (float)mqRaw;
  float flameValue = (float)flameRaw;

  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();

  bool dhtOk = !(isnan(humidity) || isnan(temperatureC));
  if (!dhtOk) {
    humidity = -1.0;
    temperatureC = -1000.0;
  }

  myData.id = DEVICE_ID;
  myData.gasAnalog = mqValue;
  myData.flameAnalog = flameValue;
  myData.temperature = temperatureC;
  myData.humidity = humidity;

  Serial.println("----- Sensor Read -----");
  Serial.printf("Device ID: %d\n", myData.id);
  Serial.printf("Gas: %.0f | Flame: %.0f | Temp: %.1f°C | Hum: %.1f%%\n",
                myData.gasAnalog, myData.flameAnalog, myData.temperature, myData.humidity);
  Serial.println("-----------------------");

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("esp_now_send(): OK");
  } else {
    Serial.printf("esp_now_send() failed: %d\n", result);
  }

  delay(2000);
}
