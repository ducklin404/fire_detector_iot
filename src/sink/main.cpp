#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Structure must match the sender exactly
typedef struct struct_message {
  int id;
  float gasAnalog;
  float flameAnalog;
  float temperature;
  float humidity;
} struct_message;

struct_message incomingData;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataBytes, int len) {
  // Print sender MAC
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("Packet received from: ");
  Serial.print(macStr);
  Serial.print("  |  Length: ");
  Serial.println(len);

  // Basic length check
  if (len != sizeof(incomingData)) {
    Serial.println("Warning: unexpected message size! Ignoring.");
    return;
  }

  // Copy bytes into struct
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  // Print content
  Serial.println("----- Sensor Data -----");
  Serial.print("Device ID: "); Serial.println(incomingData.id);
  Serial.print("MQ-5 (gas) raw: "); Serial.println(incomingData.gasAnalog);
  Serial.print("Flame raw: "); Serial.println(incomingData.flameAnalog);

  // Check for sentinel values from sender (DHT read failed)
  if (incomingData.humidity < 0 || incomingData.temperature < -100) {
    Serial.println("DHT11: Read failed on sender (sentinel values).");
  } else {
    Serial.print("DHT11 Humidity: "); Serial.print(incomingData.humidity, 1); Serial.print("%  ");
    Serial.print("Temp: "); Serial.print(incomingData.temperature, 1); Serial.println(" °C");
  }

  Serial.println("------------------------");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Starting ESP32 receiver...");

  // Print this device's MAC (verify it matches 1C:69:20:B8:C7:44)
  WiFi.mode(WIFI_STA);
  delay(50);
  Serial.print("This device MAC (STA): ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    // Optionally restart or attempt reinit
    return;
  }
  Serial.println("ESP-NOW initialized.");

  // Register to receive callbacks
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receiver ready. Waiting for data...");
}

void loop() {
  // Nothing required in loop; callback handles incoming packets.
  delay(1000);
}