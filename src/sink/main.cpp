#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#define TFLITE_SCHEMA_VERSION 3

#include "fire_logistic_int8.h"

// Data message
typedef struct struct_message {
  int id;
  float gasAnalog;
  float flameAnalog;
  float temperature;
  float humidity;
} struct_message;
struct_message incomingData;

// User config
const char* WIFI_SSID = "duck";
const char* WIFI_PASS = "ducklin404";
const char* THINGSBOARD_SERVER = "eu.thingsboard.cloud";
const uint16_t THINGSBOARD_PORT = 1883;
const char* THINGSBOARD_GATEWAY_TOKEN = "99DwCp0arbYhzG7pHere";

#define FAN_PIN 5                     
#define FAN_ACTIVE_HIGH true        

// Detection & timeout tuning
const float FIRE_THRESHOLD = 0.80f;  
const unsigned long FAN_TIMEOUT_MS = 30UL * 1000UL; 

// Globals
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
#define MAX_DEVICES 24
String seenDevices[MAX_DEVICES];
int seenCount = 0;

bool isSeen(const String &n) {
  for (int i = 0; i < seenCount; ++i) if (seenDevices[i] == n) return true;
  return false;
}
void markSeen(const String &n) {
  if (seenCount < MAX_DEVICES) seenDevices[seenCount++] = n;
}

// fan state handling
volatile unsigned long fanOnUntil = 0;  
bool fanState = false;                

// MQTT helpers
void publishDeviceConnect(const char* deviceName) {
  String payload = "{\"device\":\"";
  payload += deviceName;
  payload += "\"}";
  mqtt.publish("v1/gateway/connect", payload.c_str());
  markSeen(String(deviceName));
  Serial.printf("[MQTT] connected device %s\n", deviceName);
}

void publishTelemetry(const char* deviceName, float gas, float flame, float temp,
                      float hum, float fire_prob, bool fire_alert, bool fan_state) {
  // publish a JSON payload compatible with ThingsBoard gateway telemetry
  String payload = "{ \""; 
  payload += deviceName; 
  payload += "\":[{";
  payload += "\"gasAnalog\":"; payload += String(gas, 2);
  payload += ",\"flameAnalog\":"; payload += String(flame, 2);
  payload += ",\"temperature\":"; payload += String(temp, 2);
  payload += ",\"humidity\":"; payload += String(hum, 2);
  payload += ",\"fire_prob\":"; payload += String(fire_prob, 3);
  payload += ",\"fire_alert\":"; payload += (fire_alert ? "true" : "false");
  payload += ",\"fan_state\":"; payload += (fan_state ? "true" : "false");
  payload += "}]}";

  bool ok = mqtt.publish("v1/gateway/telemetry", payload.c_str());
  Serial.printf("[MQTT] telemetry %s -> %s\n", ok ? "SENT" : "FAILED", deviceName);
  Serial.println(payload);
}

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

// TinyML setup
const int kTensorArenaSize = 32 * 1024; 
static uint8_t tensor_arena[kTensorArenaSize];

tflite::MicroErrorReporter micro_error_reporter;
tflite::ErrorReporter* error_reporter = &micro_error_reporter;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* inputTensor = nullptr;
TfLiteTensor* outputTensor = nullptr;

bool initTFLiteModel() {
  Serial.println("[TFLM] Initializing model...");
  const tflite::Model* model = tflite::GetModel(fire_logistic_int8);
  if (!model) {
    Serial.println("[TFLM] ERROR: model pointer null");
    return false;
  }
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("[TFLM] Warning: model schema mismatch (%d vs %d)\n",
                  model->version(), TFLITE_SCHEMA_VERSION);
  }

  static tflite::AllOpsResolver resolver;

  interpreter = new tflite::MicroInterpreter(
    model, resolver, tensor_arena, static_cast<size_t>(kTensorArenaSize), error_reporter);

  if (!interpreter) {
    Serial.println("[TFLM] Interpreter allocation failed");
    return false;
  }

  TfLiteStatus status = interpreter->AllocateTensors();
  if (status != kTfLiteOk) {
    Serial.println("[TFLM] AllocateTensors() failed - try increasing tensor arena");
    return false;
  }

  inputTensor = interpreter->input(0);
  outputTensor = interpreter->output(0);
  Serial.println("[TFLM] Model ready");
  return true;
}

// Run inference on the 4 inputs in order:
//   gas, flame, temperature, humidity
// Produces outProb in range [0,1] if succeeded.
bool runInference(float gas, float flame, float temp, float hum, float &outProb) {
  if (!interpreter || !inputTensor || !outputTensor) return false;

  // basic input shape check
  int totalInputs = 1;
  for (int i = 0; i < inputTensor->dims->size; ++i) totalInputs *= inputTensor->dims->data[i];
  if (totalInputs < 4) {
    Serial.printf("[TFLM] Unexpected input size %d\n", totalInputs);
    return false;
  }

  // Handle quantized int8 vs float input
  if (inputTensor->type == kTfLiteInt8) {
    float scale = inputTensor->params.scale;
    int32_t zp = inputTensor->params.zero_point;
    if (scale == 0.0f) {
      Serial.println("[TFLM] input scale is zero!");
      return false;
    }
    int8_t *in = inputTensor->data.int8;
    in[0] = (int8_t) (round(gas / scale) + zp);
    in[1] = (int8_t) (round(flame / scale) + zp);
    in[2] = (int8_t) (round(temp / scale) + zp);
    in[3] = (int8_t) (round(hum / scale) + zp);
  } else if (inputTensor->type == kTfLiteFloat32) {
    float *in = inputTensor->data.f;
    in[0] = gas; in[1] = flame; in[2] = temp; in[3] = hum;
  } else {
    Serial.println("[TFLM] Unsupported input tensor type");
    return false;
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("[TFLM] Invoke failed");
    return false;
  }

  // Output might be quantized (int8) or float
  if (outputTensor->type == kTfLiteInt8) {
    int8_t q = outputTensor->data.int8[0];
    float s = outputTensor->params.scale;
    int32_t zp = outputTensor->params.zero_point;
    outProb = ((float)((int)q - zp)) * s;
  } else if (outputTensor->type == kTfLiteFloat32) {
    outProb = outputTensor->data.f[0];
  } else {
    Serial.println("[TFLM] Unsupported output tensor type");
    return false;
  }

  // clamp to [0,1] just to be safe
  if (outProb < 0.0f) outProb = 0.0f;
  if (outProb > 1.0f) outProb = 1.0f;
  return true;
}

// Fan utilities
static inline void setFanPin(bool on) {
  // Map logical on->pin value according to wiring
  digitalWrite(FAN_PIN, (on ^ (!FAN_ACTIVE_HIGH)) ? HIGH : LOW);
  // Explanation: If FAN_ACTIVE_HIGH==true then on->HIGH. If active low, invert.
}

void turnFanOnForTimeout() {
  // turn fan on and extend the timeout from now
  fanOnUntil = millis() + FAN_TIMEOUT_MS;
  if (!fanState) {
    fanState = true;
    setFanPin(true);
    Serial.printf("[FAN] ON (timeout %u ms)\n", (unsigned)FAN_TIMEOUT_MS);
  } else {
    // already on: just extend the timer
    Serial.printf("[FAN] Extended until +%u ms from now\n", (unsigned)FAN_TIMEOUT_MS);
  }
}

void maybeUpdateFanState() {
  // call periodically from loop() to enforce timeout
  if (fanState) {
    if (millis() >= fanOnUntil) {
      // timeout expired -> turn off
      fanState = false;
      setFanPin(false);
      Serial.println("[FAN] OFF (timeout expired)");
    }
  }
}

// ESP-NOW callback
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingBytes, int len) {
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

  // run tinyML inference
  float prob = 0.0f;
  bool ok = runInference(incomingData.gasAnalog,
                         incomingData.flameAnalog,
                         incomingData.temperature,
                         incomingData.humidity,
                         prob);

  if (!ok) {
    Serial.println("[TFLM] inference failed - publishing raw telemetry only");
    publishTelemetry(dev.c_str(),
                     incomingData.gasAnalog,
                     incomingData.flameAnalog,
                     incomingData.temperature,
                     incomingData.humidity,
                     -1.0f, false, fanState);
    return;
  }

  bool fire_alert = (prob >= FIRE_THRESHOLD);
  Serial.printf("[TFLM] fire_prob=%.3f -> alert=%s\n", prob, fire_alert ? "YES" : "no");

  // if we detect fire, turn the fan on 
  if (fire_alert) {
    turnFanOnForTimeout();
  }

  publishTelemetry(dev.c_str(),
                   incomingData.gasAnalog,
                   incomingData.flameAnalog,
                   incomingData.temperature,
                   incomingData.humidity,
                   prob, fire_alert, fanState);
}

// setup & loop
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Gateway with TinyML Fire Detection ===");

  // Init fan pin
  pinMode(FAN_PIN, OUTPUT);
  setFanPin(false); 
  fanState = false;
  fanOnUntil = 0;

  // Initialize model
  if (!initTFLiteModel()) {
    Serial.println("[TFLM] Model init failed - inference unavailable");
  }

  // initialize network + esp-now
  WiFi.mode(WIFI_STA);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed!");
  } else {
    Serial.println("[ESP-NOW] initialized");
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 50) {
    Serial.print(".");
    delay(200);
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\n[WiFi] Connected");
  else Serial.println("\n[WiFi] Failed (ESP-NOW still works)");

  mqtt.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
  if (WiFi.status() == WL_CONNECTED) mqttReconnect();

  Serial.println("[SETUP] Ready.");
}

void loop() {
  // keep MQTT alive when connected
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
  }

  maybeUpdateFanState();
  delay(10);
}
