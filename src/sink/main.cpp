#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"

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

#define FAN_PIN 32
#define FAN_ACTIVE_HIGH false
#define ALERT_PIN 33
#define ALERT_ACTIVE_HIGH false

// Detection & timeout tuning
const float FIRE_THRESHOLD = 0.80f;
const unsigned long FAN_TIMEOUT_MS = 30UL * 1000UL;

// Remembering / periodic publish config
const int NUM_SENSORS = 3;                // total sensor nodes
const float SENTINEL = -9999.0f;          // initial value meaning 'unknown'
const unsigned long PERIODIC_SEND_MS = 30UL * 1000UL; // send every 30s for each device

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

// fan & mode state handling
volatile unsigned long fanOnUntil = 0;   // used for auto timeout mode
bool fanState = false;                   // current physical fan state
bool autoMode = true;
bool manualPersistent = false;

// MQTT helpers
void publishDeviceConnect(const char* deviceName) {
  String payload = "{\"device\":\"";
  payload += deviceName;
  payload += "\"}";
  mqtt.publish("v1/gateway/connect", payload.c_str());
  markSeen(String(deviceName));
  Serial.printf("[MQTT] connected device %s\n", deviceName);
}

// Build telemetry JSON but omit fields equal to SENTINEL
void publishTelemetry(const char* deviceName, float gas, float flame, float temp,
                      float hum, float fire_prob, bool fire_alert, bool fan_state) {
  // publish a JSON payload compatible with ThingsBoard gateway telemetry
  // Format: { "<deviceName>":[{ "key":value, ... }] }
  String payload = "{ \"";
  payload += deviceName;
  payload += "\":[{";

  bool first = true;
  auto appendKV = [&](const char* k, const String &v) {
    if (!first) payload += ",";
    payload += "\"";
    payload += k;
    payload += "\":";
    payload += v;
    first = false;
  };

  if (gas != SENTINEL) appendKV("gasAnalog", String(gas, 2));
  if (flame != SENTINEL) appendKV("flameAnalog", String(flame, 2));
  if (temp != SENTINEL) appendKV("temperature", String(temp, 2));
  if (hum != SENTINEL) appendKV("humidity", String(hum, 2));

  // include fire_prob if known (>= 0), else omit or set -1
  if (fire_prob >= 0.0f) appendKV("fire_prob", String(fire_prob, 3));
  else appendKV("fire_prob", String(-1.0, 1));

  appendKV("fire_alert", (fire_alert ? "true" : "false"));
  appendKV("fan_state", (fan_state ? "true" : "false"));
  appendKV("auto_mode", (autoMode ? "true" : "false"));
  appendKV("manual_control_allowed", (!autoMode ? "true" : "false"));

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
      mqtt.subscribe("v1/devices/me/rpc/request/+");
      Serial.println("[MQTT] Subscribed to v1/gateway/rpc");
      break;
    } else {
      Serial.printf(" failed, rc=%d. retrying...\n", mqtt.state());
      delay(2000);
    }
  }
}

// TinyML setup (unchanged)
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

// Fan utilities (unchanged)
static inline void setFanPin(bool on) {
  if (FAN_ACTIVE_HIGH) digitalWrite(FAN_PIN, on ? HIGH : LOW);
  else digitalWrite(FAN_PIN, on ? LOW : HIGH);
}

static inline void setAlertPin(bool on) {
  if (ALERT_ACTIVE_HIGH) digitalWrite(ALERT_PIN, on ? HIGH : LOW);
  else digitalWrite(ALERT_PIN, on ? LOW : HIGH);
}

void turnFanOnForTimeout() {
  fanOnUntil = millis() + FAN_TIMEOUT_MS;
  if (!fanState) {
    fanState = true;
    setFanPin(true);
    Serial.printf("[FAN] ON (auto timeout %u ms)\n", (unsigned)FAN_TIMEOUT_MS);
  } else {
    Serial.printf("[FAN] Extended until +%u ms from now\n", (unsigned)FAN_TIMEOUT_MS);
  }
}

void setFanManualPersistent(bool on) {
  manualPersistent = true;
  fanOnUntil = 0;
  if (fanState != on) {
    fanState = on;
    setFanPin(on);
    Serial.printf("[FAN] Manual persistent set to %s\n", on ? "ON" : "OFF");
  } else {
    Serial.printf("[FAN] Manual persistent already %s\n", on ? "ON" : "OFF");
  }
}

void maybeUpdateFanState() {
  if (fanState) {
    if (autoMode && fanOnUntil > 0) {
      if (millis() >= fanOnUntil) {
        fanState = false;
        setFanPin(false);
        setAlertPin(false);
        Serial.println("[FAN] OFF (auto timeout expired)");
      }
    }
    if (!autoMode && manualPersistent) {
      // nothing to do here
    }
  }
}

void publishRpcResponse(const String &requestId, const String &responseJson) {
  if (requestId.length() == 0) return;
  String topic = "v1/gateway/rpc/response/" + requestId;
  mqtt.publish(topic.c_str(), responseJson.c_str());
  Serial.printf("[MQTT] RPC response to id=%s -> %s\n", requestId.c_str(), responseJson.c_str());
}

// Simple JSON helpers (unchanged)
String extractJsonStringValue(const String &json, const String &key) {
  String pattern = "\"" + key + "\"";
  int idx = json.indexOf(pattern);
  if (idx < 0) return String();
  int colon = json.indexOf(':', idx + pattern.length());
  if (colon < 0) return String();
  int start = json.indexOf('\"', colon);
  if (start < 0) return String();
  int end = json.indexOf('\"', start + 1);
  if (end < 0) return String();
  return json.substring(start + 1, end);
}
String extractJsonRawValue(const String &json, const String &key) {
  String pattern = "\"" + key + "\"";
  int idx = json.indexOf(pattern);
  if (idx < 0) return String();
  int colon = json.indexOf(':', idx + pattern.length());
  if (colon < 0) return String();
  int i = colon + 1;
  while (i < (int)json.length() && isspace((uint8_t)json[i])) i++;
  if (i >= (int)json.length()) return String();
  if (json[i] == '"') {
    int start = i;
    int end = json.indexOf('"', start + 1);
    if (end < 0) return String();
    return json.substring(start, end + 1);
  } else {
    int j = i;
    while (j < (int)json.length() && json[j] != ',' && json[j] != '}' && json[j] != ']') j++;
    return json.substring(i, j);
  }
}
bool parseBoolToken(const String &token, bool &ok) {
  ok = true;
  String t = token;
  t.trim();
  if (t.length() == 0) { ok = false; return false; }
  if (t[0] == '\"' && t[t.length()-1] == '\"') t = t.substring(1, t.length()-1);
  t.toLowerCase();
  if (t == "true" || t == "1") return true;
  if (t == "false" || t == "0") return false;
  ok = false;
  return false;
}

// MQTT callback for RPC (unchanged)
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.printf("[MQTT] Message arrived on topic %s: %s\n", topic, msg.c_str());

  String requestId = extractJsonRawValue(msg, "id");
  requestId.trim();
  if (requestId.startsWith("\"") && requestId.endsWith("\"")) {
    requestId = requestId.substring(1, requestId.length() - 1);
  }

  String method = extractJsonStringValue(msg, "method");
  String paramsRaw = extractJsonRawValue(msg, "params");

  Serial.printf("[MQTT] RPC method=%s params=%s id=%s\n", method.c_str(), paramsRaw.c_str(), requestId.c_str());

  String response = "{\"result\":\"unsupported method\"}";

  if (method.equalsIgnoreCase("setMode")) {
    bool ok;
    bool val = parseBoolToken(paramsRaw, ok);
    if (!ok) {
      response = "{\"error\":\"invalid params for setMode\"}";
    } else {
      autoMode = val;
      if (autoMode) {
        manualPersistent = false;
        Serial.println("[RPC] setMode -> AUTO ON");
      } else {
        Serial.println("[RPC] setMode -> AUTO OFF (manual control allowed)");
      }
      response = String("{\"success\":true, \"auto_mode\":") + (autoMode ? "true" : "false") + "}";
    }
  } else if (method.equalsIgnoreCase("setFan")) {
    bool ok;
    bool val = parseBoolToken(paramsRaw, ok);
    if (!ok) {
      response = "{\"error\":\"invalid params for setFan\"}";
    } else {
      if (!autoMode) {
        setFanManualPersistent(val);
        response = String("{\"success\":true, \"fan_state\":") + (fanState ? "true" : "false") + "}";
      } else {
        response = "{\"error\":\"manual control disabled while auto mode is ON\"}";
        Serial.println("[RPC] Ignored setFan because auto mode is ON");
      }
    }
  } else {
    Serial.printf("[RPC] Unknown method: %s\n", method.c_str());
    response = "{\"error\":\"unknown method\"}";
  }

  if (requestId.length() > 0) {
    publishRpcResponse(requestId, response);
  }
}

// Storage for last-known data for each sensor node
struct_message lastKnown[NUM_SENSORS + 1]; // index by device id (1..NUM_SENSORS)
unsigned long lastPeriodicSend = 0;

// Helper: check if any field known for device
bool anyFieldKnown(const struct_message &m) {
  return (m.gasAnalog != SENTINEL) || (m.flameAnalog != SENTINEL) ||
         (m.temperature != SENTINEL) || (m.humidity != SENTINEL);
}
// Helper: check if all fields known for device
bool allFieldsKnown(const struct_message &m) {
  return (m.gasAnalog != SENTINEL) && (m.flameAnalog != SENTINEL) &&
         (m.temperature != SENTINEL) && (m.humidity != SENTINEL);
}

// Process incoming ESP-NOW packet (unchanged behavior + update lastKnown)
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

  int id = incomingData.id;
  if (id < 1 || id > NUM_SENSORS) {
    Serial.printf("[ESP-NOW] Ignoring unknown device id=%d\n", id);
    return;
  }

  String dev = "sensor_" + String(id);
  Serial.printf("[DATA] %s gas=%.2f flame=%.2f temp=%.2f hum=%.2f\n",
                dev.c_str(),
                incomingData.gasAnalog,
                incomingData.flameAnalog,
                incomingData.temperature,
                incomingData.humidity);

  // update lastKnown fields only when incoming value isn't sentinel
  if (incomingData.gasAnalog != SENTINEL) lastKnown[id].gasAnalog = incomingData.gasAnalog;
  if (incomingData.flameAnalog != SENTINEL) lastKnown[id].flameAnalog = incomingData.flameAnalog;
  if (incomingData.temperature != SENTINEL) lastKnown[id].temperature = incomingData.temperature;
  if (incomingData.humidity != SENTINEL) lastKnown[id].humidity = incomingData.humidity;

  // MQTT connect if needed
  if (!mqtt.connected()) mqttReconnect();
  if (!isSeen(dev)) {
    publishDeviceConnect(dev.c_str());
    delay(100);
  }

  // run tinyML inference on this fresh incoming packet if possible
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
  } else {
    bool fire_alert = (prob >= FIRE_THRESHOLD);
    Serial.printf("[TFLM] fire_prob=%.3f -> alert=%s\n", prob, fire_alert ? "YES" : "no");

    // safety: if fire -> force fan + alarm
    if (fire_alert) {
      Serial.println("[ALERT] Fire detected -> forcing fan ON for timeout and enabling alarm");
      turnFanOnForTimeout();
      setAlertPin(true);
    }

    // auto mode temp rule (applies only on incoming fresh messages)
    if (autoMode) {
      if (incomingData.temperature > 30.0f) {
        Serial.printf("[AUTO] temp %.2f > 30C -> turning fan ON (autoMode)\n", incomingData.temperature);
        turnFanOnForTimeout();
      }
    }

    publishTelemetry(dev.c_str(),
                     incomingData.gasAnalog,
                     incomingData.flameAnalog,
                     incomingData.temperature,
                     incomingData.humidity,
                     prob, fire_alert, fanState);
  }
}

// setup & loop
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Gateway with TinyML Fire Detection (remember last-known, periodic ThingsBoard publish) ===");

  // Init fan & alert pins
  pinMode(FAN_PIN, OUTPUT);
  setFanPin(false);
  fanState = false;
  fanOnUntil = 0;

  pinMode(ALERT_PIN, OUTPUT);
  setAlertPin(false);

  // Initialize model
  if (!initTFLiteModel()) {
    Serial.println("[TFLM] Model init failed - inference unavailable");
  }

  // initialize lastKnown with sentinel
  for (int i = 1; i <= NUM_SENSORS; ++i) {
    lastKnown[i].id = i;
    lastKnown[i].gasAnalog = SENTINEL;
    lastKnown[i].flameAnalog = SENTINEL;
    lastKnown[i].temperature = SENTINEL;
    lastKnown[i].humidity = SENTINEL;
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
  mqtt.setCallback(mqttCallback);
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

  // Periodically send telemetry for all known devices (even if they haven't just sent)
  unsigned long now = millis();
  if (now - lastPeriodicSend >= PERIODIC_SEND_MS) {
    lastPeriodicSend = now;
    // If mqtt disconnected, attempt reconnect (mqttReconnect called above if WiFi ok)
    for (int id = 1; id <= NUM_SENSORS; ++id) {
      String dev = "sensor_" + String(id);
      // only send telemetry for sensors that have at least one known field
      if (!anyFieldKnown(lastKnown[id])) {
        // console debug
        Serial.printf("[PERIODIC] skipping %s (no known data yet)\n", dev.c_str());
        continue;
      }

      if (!isSeen(dev)) {
        // Ensure ThingsBoard knows the device
        if (!mqtt.connected()) mqttReconnect();
        publishDeviceConnect(dev.c_str());
        delay(50);
      }

      // If all fields known, run inference; otherwise set prob to -1 (unknown)
      float prob = -1.0f;
      bool fire_alert = false;
      if (allFieldsKnown(lastKnown[id]) && interpreter) {
        bool ok = runInference(lastKnown[id].gasAnalog,
                               lastKnown[id].flameAnalog,
                               lastKnown[id].temperature,
                               lastKnown[id].humidity,
                               prob);
        if (ok) {
          fire_alert = (prob >= FIRE_THRESHOLD);
        } else {
          prob = -1.0f;
        }
      } else {
        prob = -1.0f;
      }

      // For periodic publishes we do NOT auto-turn the fan on/off based on old data:
      // we only publish telemetry (to avoid spuriously toggling outputs).
      if (!mqtt.connected()) mqttReconnect();
      publishTelemetry(dev.c_str(),
                       lastKnown[id].gasAnalog,
                       lastKnown[id].flameAnalog,
                       lastKnown[id].temperature,
                       lastKnown[id].humidity,
                       prob, fire_alert, fanState);
    }
  }

  delay(10);
}
