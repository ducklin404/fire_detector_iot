#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>
#include "esp_wifi.h"

const int MQ5_PIN = 33;
const int FLAME_PIN = 35;
const int DHT_PIN = 23;
const int DEVICE_ID = 2; 

float gasEMA = NAN;
float flameEMA = NAN;
float tempEMA = NAN;
float humidEMA = NAN;

// EMA alpha
const float gasAlpha = 0.30f;
const float flameAlpha = 0.25f;
const float tempAlpha = 0.15f;
const float humidAlpha = 0.20f;

// --- Range thresholds (ABSOLUTE) ---
// Node will send when EMA value is outside lastSent +/- RANGE
const float GAS_RANGE = 15.0f;     
const float FLAME_RANGE = 15.0f;   
const float TEMP_RANGE = 0.5f;     
const float HUMID_RANGE = 2.0f;    


// optional: minimum time between sends (ms)
const unsigned long MIN_SEND_INTERVAL = 5000UL;

uint8_t broadcastAddress[] = {0x1C, 0x69, 0x20, 0xB8, 0xC7, 0x44};

// Force both sender and gateway to same channel
#define ESPNOW_CHANNEL 6

typedef struct struct_message {
  int id;
  float gasAnalog;   // EMA value
  float flameAnalog; // EMA value
  float temperature; // EMA value (C)
  float humidity;    // EMA value (%)
} struct_message;

struct_message myData;

#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

unsigned long lastSendTime = 0;
// last values we actually transmitted
float lastSentGas = NAN;
float lastSentFlame = NAN;
float lastSentTemp = NAN;
float lastSentHumid = NAN;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting ESP32 sensor sender (EMA + range thresholds)...");

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

inline bool exceedsRangeAbs(float current, float lastSent, float range) {
  if (isnan(lastSent)) return true; // first send
  return fabs(current - lastSent) > range;
}

inline bool exceedsRangePct(float current, float lastSent, float pctThreshold) {
  if (isnan(lastSent) || lastSent == 0.0f) return true; // fallback to absolute behavior first time
  float pct = fabs((current - lastSent) / lastSent) * 100.0f;
  return pct >= pctThreshold;
}

void loop() {
  int mqRaw = analogRead(MQ5_PIN);
  int flameRaw = analogRead(FLAME_PIN);

  float mqValue = (float)mqRaw;
  float flameValue = (float)flameRaw;

  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();

  // apply EMA - initialize with first valid reading
  if (isnan(gasEMA)) {
    gasEMA = mqValue;
  } else {
    gasEMA = gasAlpha * mqValue + (1.0f - gasAlpha) * gasEMA;
  }

  if (isnan(flameEMA)) {
    flameEMA = flameValue;
  } else {
    flameEMA = flameAlpha * flameValue + (1.0f - flameAlpha) * flameEMA;
  }

  bool dhtOk = !(isnan(humidity) || isnan(temperatureC));
  if (dhtOk) {
    if (isnan(tempEMA)) tempEMA = temperatureC;
    else tempEMA = tempAlpha * temperatureC + (1.0f - tempAlpha) * tempEMA;

    if (isnan(humidEMA)) humidEMA = humidity;
    else humidEMA = humidAlpha * humidity + (1.0f - humidAlpha) * humidEMA;
  } else {
    // If DHT reading failed, we leave tempEMA/humidEMA unchanged (no update).
    // Optionally you could set them to NaN/sentinel or set flags.
    Serial.println("Warning: DHT read failed this cycle");
  }

  // Prepare EMA values to send
  float sendGas = gasEMA;
  float sendFlame = flameEMA;
  float sendTemp = tempEMA;   
  float sendHumid = humidEMA;

  // Decide whether to send:
  bool shouldSend = false;

  // If first time (lastSent NaN) and we have a value (not NaN), send.
  if (isnan(lastSentGas) && !isnan(sendGas)) shouldSend = true;
  if (isnan(lastSentFlame) && !isnan(sendFlame)) shouldSend = true;
  if (isnan(lastSentTemp) && !isnan(sendTemp)) shouldSend = true;
  if (isnan(lastSentHumid) && !isnan(sendHumid)) shouldSend = true;

  if (!shouldSend) {
    // absolute range mode
    if (!isnan(sendGas) && exceedsRangeAbs(sendGas, lastSentGas, GAS_RANGE)) shouldSend = true;
    if (!isnan(sendFlame) && exceedsRangeAbs(sendFlame, lastSentFlame, FLAME_RANGE)) shouldSend = true;
    if (!isnan(sendTemp) && exceedsRangeAbs(sendTemp, lastSentTemp, TEMP_RANGE)) shouldSend = true;
    if (!isnan(sendHumid) && exceedsRangeAbs(sendHumid, lastSentHumid, HUMID_RANGE)) shouldSend = true;
  }

  // Enforce minimum send interval to limit bursty sends
  unsigned long now = millis();
  if ((now - lastSendTime) < MIN_SEND_INTERVAL) {
    shouldSend = false;
  }

  if (shouldSend) {
    myData.id = DEVICE_ID;
    myData.gasAnalog = sendGas;
    myData.flameAnalog = sendFlame;
    myData.temperature = isnan(sendTemp) ? -1000.0f : sendTemp; // sentinel for receiver
    myData.humidity = isnan(sendHumid) ? -1.0f : sendHumid;

    Serial.println("----- EMA Sensor Read (to be sent) -----");
    Serial.printf("Device ID: %d\n", myData.id);
    Serial.printf("Gas (EMA): %.1f | Flame (EMA): %.1f | Temp (EMA): %.2f°C | Hum (EMA): %.2f%%\n",
                  myData.gasAnalog, myData.flameAnalog, myData.temperature, myData.humidity);
    Serial.println("-----------------------");

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("esp_now_send(): OK");
      // update last sent values and time only on success
      if (!isnan(sendGas)) lastSentGas = sendGas;
      if (!isnan(sendFlame)) lastSentFlame = sendFlame;
      if (!isnan(sendTemp)) lastSentTemp = sendTemp;
      if (!isnan(sendHumid)) lastSentHumid = sendHumid;
      lastSendTime = now;
    } else {
      Serial.printf("esp_now_send() failed: %d\n", result);
    }
  } else {
    // debug 
    Serial.println("No significant EMA change -> not sending to save bandwidth.");
    Serial.printf("Gas EMA: %.1f (last sent: %.1f) | Flame EMA: %.1f (last sent: %.1f)\n",
                  sendGas, isnan(lastSentGas) ? -1.0f : lastSentGas,
                  sendFlame, isnan(lastSentFlame) ? -1.0f : lastSentFlame);
    Serial.printf("Temp EMA: %.2f (last sent: %.2f) | Hum EMA: %.2f (last sent: %.2f)\n",
                  isnan(sendTemp) ? -1000.0f : sendTemp, isnan(lastSentTemp) ? -1000.0f : lastSentTemp,
                  isnan(sendHumid) ? -1.0f : sendHumid, isnan(lastSentHumid) ? -1.0f : lastSentHumid);
  }

  // sensor read period
  delay(2000);
}
