#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"

// ========== DATA STRUCTURE ==========
typedef struct struct_message {
  int id;
  float gasAnalog;
  float flameAnalog;
  float temperature;
  float humidity;
} struct_message;
struct_message incomingData;

// USER CONFIG
const char* WIFI_SSID = "duck";
const char* WIFI_PASS = "ducklin404";
const char* THINGSBOARD_SERVER = "eu.thingsboard.cloud";
const uint16_t THINGSBOARD_PORT = 1883;
const char* THINGSBOARD_GATEWAY_TOKEN = "99DwCp0arbYhzG7pHere"; // gateway token

// GLOABL VARIABLES
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
#define MAX_DEVICES 24
String seenDevices[MAX_DEVICES];
int seenCount = 0;

bool isSeen(const String &n) {
  for (int i=0;i<seenCount;i++) if (seenDevices[i]==n) return true;
  return false;
}
void markSeen(const String &n) {
  if (seenCount < MAX_DEVICES) seenDevices[seenCount++] = n;
}

// publish mqtt
void publishDeviceConnect(const char* deviceName) {
  String payload = "{\"device\":\""; payload += deviceName; payload += "\"}";
  mqtt.publish("v1/gateway/connect", payload.c_str());
  markSeen(String(deviceName));
  Serial.printf("[MQTT] connected device %s\n", deviceName);
}

void publishTelemetry(const char* deviceName, float gas, float flame, float temp, float hum) {
  String payload = "{ \""; 
  payload += deviceName; 
  payload += "\":[{";
  payload += "\"gasAnalog\":"; payload += String(gas, 2);
  payload += ",\"flameAnalog\":"; payload += String(flame, 2);
  payload += ",\"temperature\":"; payload += String(temp, 2);
  payload += ",\"humidity\":"; payload += String(hum, 2);
  payload += "}]}";

  bool ok = mqtt.publish("v1/gateway/telemetry", payload.c_str());
  Serial.printf("[MQTT] telemetry %s -> %s\n", ok ? "SENT" : "FAILED", deviceName);
  Serial.println(payload);
}

// mqtt reconnect
void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Connecting...");
    String clientId = "esp32_gw_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(clientId.c_str(), THINGSBOARD_GATEWAY_TOKEN, NULL)) {
      Serial.println(" connected");
      break;
    } else {
      Serial.printf(" failed, rc=%d. retrying...\n", mqtt.state());
      delay(2000);
    }
  }
}

// ESP NOW CALLBACK
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingBytes, int len) {
  Serial.println("\n[ESP-NOW] Packet received");
  char macStr[18];
  snprintf(macStr,sizeof(macStr),"%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.printf("From MAC: %s len=%d\n", macStr, len);

  if (len != sizeof(incomingData)) { Serial.println("[ESP-NOW] Wrong size."); return; }
  memcpy(&incomingData, incomingBytes, sizeof(incomingData));

  String dev = "sensor_" + String(incomingData.id);
  Serial.printf("[DATA] %s gas=%.2f flame=%.2f temp=%.2f hum=%.2f\n",
                dev.c_str(),
                incomingData.gasAnalog,
                incomingData.flameAnalog,
                incomingData.temperature,
                incomingData.humidity);

  if (!mqtt.connected()) mqttReconnect();

  if (!isSeen(dev)) {
    publishDeviceConnect(dev.c_str());
    delay(100);
  }

  publishTelemetry(dev.c_str(),
                   incomingData.gasAnalog,
                   incomingData.flameAnalog,
                   incomingData.temperature,
                   incomingData.humidity);
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 GATEWAY START ===");

  WiFi.mode(WIFI_STA);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed!");
  } else {
    Serial.println("[ESP-NOW] Initialized");
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries=0;
  while (WiFi.status()!=WL_CONNECTED && tries<50){ Serial.print("."); delay(200); tries++; }
  if (WiFi.status()==WL_CONNECTED) Serial.println("\n[WiFi] Connected");
  else Serial.println("\n[WiFi] Failed (ESP-NOW still works)");

  mqtt.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
  if (WiFi.status()==WL_CONNECTED) mqttReconnect();

  Serial.println("[SETUP] Ready.");
}

void loop() {
  if (WiFi.status()==WL_CONNECTED) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
  }
  delay(10);
}
