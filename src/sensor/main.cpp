#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

// Pin definitions
const int MQ5_PIN = 33;     
const int FLAME_PIN = 35;   
const int DHT_PIN = 23;    
const int DEVICE_ID = 2;

// receiver MAC (your provided address)
uint8_t broadcastAddress[] = {0x1C, 0x69, 0x20, 0xB8, 0xC7, 0x44};

typedef struct struct_message {
  int id;
  float gasAnalog;
  float flameAnalog;
  float temperature;
  float humidity;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// DHT setup
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

// ADC reading parameters
const int ADC_MAX = 4095; 

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Starting ESP32 sensor sender...");

  // WiFi in STA mode (radio ON, not connected to any AP)
  WiFi.mode(WIFI_STA);
  delay(100);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    // If initialization fails, don't continue — you might want to reset or retry
    return;
  }
  Serial.println("ESP-NOW initialized.");

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register receiver as peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;    // use current WiFi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.print("Added peer: ");
    for (int i = 0; i < 6; i++) {
      if (i) Serial.print(":");
      Serial.print(broadcastAddress[i], HEX);
    }
    Serial.println();
  } else {
    Serial.println("Failed to add peer. Check MAC address and try again.");
    // continue; you may still be able to send using broadcast in some cases
  }

  // Initialize DHT
  dht.begin();

  // Configure ADC attenuation for full range reading
  analogSetPinAttenuation(MQ5_PIN, ADC_11db);
  analogSetPinAttenuation(FLAME_PIN, ADC_11db);

  Serial.println("Give sensors a few seconds to stabilize...");
  delay(2000);
}

void loop() {
  // Read raw ADCs 
  int mqRaw = analogRead(MQ5_PIN);
  int flameRaw = analogRead(FLAME_PIN);

  // Normalize to 0..1 if you want (example), but we'll send raw values as floats
  float mqValue = (float)mqRaw;
  float flameValue = (float)flameRaw;

  // Read DHT (temperature & humidity)
  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature(); 

  // Check for DHT read errors (NaN)
  bool dhtOk = !(isnan(humidity) || isnan(temperatureC));
  if (!dhtOk) {
    // set sentinel values so receiver knows read failed
    humidity = -1.0;
    temperatureC = -1000.0;
  }

  // Fill the struct
  myData.id = DEVICE_ID;
  myData.gasAnalog = mqValue;
  myData.flameAnalog = flameValue;
  myData.temperature = temperatureC;
  myData.humidity = humidity;

  // Print locally for debugging
  Serial.println("----- sensor read -----");
  Serial.print("Device ID: "); Serial.println(myData.id);
  Serial.print("MQ-5 raw: "); Serial.println(mqRaw);
  Serial.print("Flame raw: "); Serial.println(flameRaw);
  if (dhtOk) {
    Serial.print("DHT11 Humidity: "); Serial.print(humidity, 1); Serial.print("%  ");
    Serial.print("Temp: "); Serial.print(temperatureC, 1); Serial.println(" °C");
  } else {
    Serial.println("DHT11: Read failed (NaN). Sending sentinel values.");
  }
  Serial.println("-----------------------");

  // Send via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("esp_now_send(): OK (queued)");
  } else {
    Serial.print("esp_now_send() failed with error: ");
    Serial.println(result);
  }

  // Wait before next read/send
  delay(2000); // 2 seconds
}
