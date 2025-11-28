#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "config.h"
#include "types.h"
#include "utils.h"
#include "espnow_handler.h"
#include "mqtt_client.h"
#include "fan_control.h"
#include "mqtt.h"


// constants from former global config
const char *WIFI_SSID = "Ace 5 Pro";
const char *WIFI_PASS = "hupk4928";
const char *THINGSBOARD_SERVER = "eu.thingsboard.cloud";
const uint16_t THINGSBOARD_PORT = 1883;
const char *THINGSBOARD_GATEWAY_TOKEN = "lr5g7fevivn9gp2dlhwx";

static bool auto_fan = false;   // if true, fan is controlled automatically
static bool auto_alarm = false; // if true, alarm is controlled automatically (auto_mode now only affects alarm)
static bool fan_on = false;     // manual persistent fan desired state (when auto_fan == false)
static bool alarm_on = false;   // manual persistent alarm desired state (when auto_alarm == false)
static const int ESPNOW_CHANNEL = 6;

// mutable thresholds (user-specified via attributes)
float GAS_THRESHOLD = 400.0;
float FLAME_THRESHOLD = 2000.0f;
float HUMID_THRESHOLD = 55.0f;
const unsigned long FAN_TIMEOUT_MS = 30UL * 1000UL;

const int NUM_SENSORS = 2;
const float SENTINEL = -9999.0f;
const unsigned long PERIODIC_SEND_MS = 30UL * 1000UL;

enum FanMode
{
  FAN_MANUAL = 0,
  FAN_AUTO = 1
};
static FanMode fanMode = FAN_MANUAL;
static unsigned long lastSensorTimestamp[11] = {0};
const unsigned long SENSOR_STALE_MS = 2UL * 60UL * 1000UL;

// last-known storage
static struct_message lastKnown[10]; // NUM_SENSORS small
static unsigned long lastPeriodicSend = 0;

bool badAir(float gas_analog, float humid_analog)
{
  if (gas_analog >= GAS_THRESHOLD || humid_analog >= HUMID_THRESHOLD)
  {
    return true;
  }
  return false;
}

static bool anySensorBadAir()
{
  for (int i = 1; i <= NUM_SENSORS; ++i)
  {
    if (lastKnown[i].gasAnalog != SENTINEL || lastKnown[i].humidity != SENTINEL)
    {
      // if we track timestamps, can check staleness:
      // if (millis() - lastSensorTimestamp[i] > SENSOR_STALE_MS) continue;
      if (badAir(lastKnown[i].gasAnalog, lastKnown[i].humidity))
        return true;
    }
  }
  return false;
}

static bool detectFireFromReadings(float flame)
{
  // fire when flame reading crosses threshold
  if (flame <= FLAME_THRESHOLD)
    return true;
  return false;
}

static bool anySensorFire()
{
  for (int i = 1; i <= NUM_SENSORS; ++i)
  {
    if (lastKnown[i].flameAnalog != SENTINEL)
    {
      // optional staleness check could go here:
      // if (millis() - lastSensorTimestamp[i] > SENSOR_STALE_MS) continue;
      if (detectFireFromReadings(lastKnown[i].flameAnalog))
        return true;
    }
  }
  return false;
}

static void evaluateAutoAlarm()
{
  if (!auto_alarm)
    return; // only relevant in auto mode

  // Do we have any flame data at all?
  bool haveAnyData = false;
  for (int i = 1; i <= NUM_SENSORS; ++i)
  {
    if (lastKnown[i].flameAnalog != SENTINEL || lastKnown[i].gasAnalog != SENTINEL)
    {
      haveAnyData = true;
      break;
    }
  }

  if (!haveAnyData)
  {
    // No recent data -> safe default: keep alarm OFF
    // (You can choose to keep previous state, but turning alarm on with no data is dangerous)
    turnAlarmOff();
    return;
  }

  // If any sensor reports fire -> alarm on, otherwise off
  if (anySensorFire())
    turnAlarmOn();
  else
    turnAlarmOff();
}

static void evaluateAutoFan()
{
  if (!auto_fan)
    return; // only for auto mode

  // If we have ANY recent data, decide from it; otherwise choose safe default.
  bool haveAnyData = false;
  for (int i = 1; i <= NUM_SENSORS; ++i)
  {
    if (lastKnown[i].gasAnalog != SENTINEL || lastKnown[i].humidity != SENTINEL ||
        lastKnown[i].flameAnalog != SENTINEL || lastKnown[i].temperature != SENTINEL)
    {
      haveAnyData = true;
      break;
    }
  }

  if (!haveAnyData)
  {
    // No recent sensor data.  explicitly turn fan OFF
    turnFanOff();
    return;
  }

  // Decide using latest knowns
  if (anySensorBadAir())
    turnFanOn();
  else
    turnFanOff();
}



String rpcHandler(const String &key, const String &value)
{

  Serial.println(key);
  Serial.println(value);

  if (key.equalsIgnoreCase("auto_fan"))
  {
    bool ok;
    bool val = parseBoolToken(value, ok);
    // set mode
    auto_fan = val;
    fanMode = auto_fan ? FAN_AUTO : FAN_MANUAL;

    if (auto_fan)
    {
      evaluateAutoFan();
    }
    else
    {
      // apply the manual persisted desired state immediately
      if (fan_on)
        turnFanOn();
      else
        turnFanOff();
    }
  }
  else if (key.equalsIgnoreCase("fan_on"))
  {
    bool ok;
    bool val = parseBoolToken(value, ok);
    fan_on = val;
    if (!auto_fan)
    {
      if (fan_on)
      {
        turnFanOn();
      }
      else
      {
        turnFanOff();
      }
    }
  }
  else if (key.equalsIgnoreCase("auto_alarm"))
  {
    bool ok;
    bool val = parseBoolToken(value, ok);
    // set auto_alarm mode
    auto_alarm = val;

    if (auto_alarm)
    {
      // Immediately apply auto decision using lastKnown[] (do not wait)
      evaluateAutoAlarm();
    }
    else
    {
      // Switching to manual: apply manual persisted desired state immediately
      if (alarm_on)
        turnAlarmOn();
      else
        turnAlarmOff();
    }
  }
  else if (key.equalsIgnoreCase("alarm_on"))
  {
    bool ok;
    bool val = parseBoolToken(value, ok);
    alarm_on = val;
    if (!auto_alarm)
    {
      if (alarm_on)
      {
        turnAlarmOn();
      }
      else
      {
        turnAlarmOff();
      }
    }
  }
  else if (key.equalsIgnoreCase("gas_threshold"))
  {
    bool ok;
    float val = parseFloatToken(value, ok);
    GAS_THRESHOLD = val;
    if (auto_fan){
      evaluateAutoFan();
    }
  }
  else if (key.equalsIgnoreCase("humid_threshold"))
  {
    bool ok;
    float val = parseFloatToken(value, ok);
    HUMID_THRESHOLD = val;
    if (auto_fan){
      evaluateAutoFan();
    }
  }
  return "Done";
}

static void onSensorData(const struct_message &msg)
{
  int id = msg.id;
  if (id < 1 || id > NUM_SENSORS)
  {
    Serial.printf("[ESP-NOW] Ignoring unknown device id=%d\n", id);
    return;
  }
  String dev = "sensor_" + String(id);

  Serial.printf("[DATA] %s gas=%.2f flame=%.2f temp=%.2f hum=%.2f\n",
                dev.c_str(), msg.gasAnalog, msg.flameAnalog, msg.temperature, msg.humidity);

  if (msg.gasAnalog != SENTINEL) {
    lastKnown[id].gasAnalog = msg.gasAnalog;
    lastSensorTimestamp[id] = millis();
  }
  if (msg.flameAnalog != SENTINEL) {
    lastKnown[id].flameAnalog = msg.flameAnalog;
    lastSensorTimestamp[id] = millis();
  }
  if (msg.temperature != SENTINEL) {
    lastKnown[id].temperature = msg.temperature;
    lastSensorTimestamp[id] = millis();
  }
  if (msg.humidity != SENTINEL) {
    lastKnown[id].humidity = msg.humidity;
    lastSensorTimestamp[id] = millis();
  }

  // ensure MQTT connected and send telemetry
  mqttEnsureConnected();
  // if first time, let publishDeviceConnect handle it (it stores seen devices internally)
  mqttPublishDeviceConnect(dev.c_str());

  bool fire_alert = detectFireFromReadings(msg.flameAnalog);
  Serial.printf("[DETECT] fire_alert=%s (flame>=%.2f gas>=%.2f temp>=%.2f)\n",
                fire_alert ? "YES" : "no", FLAME_THRESHOLD, GAS_THRESHOLD, HUMID_THRESHOLD);

  if (auto_alarm) evaluateAutoAlarm();
  if (auto_fan) evaluateAutoFan();

  mqttPublishTelemetry(dev.c_str(), msg.gasAnalog, msg.flameAnalog, msg.temperature, msg.humidity, fire_alert, false);
}



void setup()
{
  Serial.begin(115200);
  Serial.println("=== ESP32 Gateway (refactored) ===");

  fanInit();

  // init lastKnown
  for (int i = 1; i <= NUM_SENSORS; ++i)
  {
    lastKnown[i].id = i;
    lastKnown[i].gasAnalog = SENTINEL;
    lastKnown[i].flameAnalog = SENTINEL;
    lastKnown[i].temperature = SENTINEL;
    lastKnown[i].humidity = SENTINEL;
  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());   

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("STA MAC bytes: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  espnowInit(onSensorData);

  // start WiFi (for MQTT); allow ESP-NOW to operate without WiFi connected
  Serial.printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 50)
  {
    Serial.print(".");
    delay(200);
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED){
    Serial.println("\n[WiFi] Connected");
      Serial.printf("[WiFi] Connected channel: %d\n", WiFi.channel());
  }
  else
    Serial.println("\n[WiFi] Failed (ESP-NOW still works)");

  // mqtt
  mqttInit();
  mqttSetRpcHandler(rpcHandler);
  if (WiFi.status() == WL_CONNECTED)
    mqttEnsureConnected();
  
  Serial.println("[SETUP] Ready.");
}

void loop()
{
  // mqtt loop
  mqttLoop();

  // periodic telemetry for devices that have any known field
  unsigned long now = millis();
  if (now - lastPeriodicSend >= PERIODIC_SEND_MS)
  {
    lastPeriodicSend = now;
    for (int id = 1; id <= NUM_SENSORS; ++id)
    {
      String dev = "sensor_" + String(id);
      bool anyKnown = (lastKnown[id].gasAnalog != SENTINEL) ||
                      (lastKnown[id].flameAnalog != SENTINEL) ||
                      (lastKnown[id].temperature != SENTINEL) ||
                      (lastKnown[id].humidity != SENTINEL);
      if (!anyKnown)
      {
        Serial.printf("[PERIODIC] skipping %s (no known data yet)\n", dev.c_str());
        continue;
      }

      mqttEnsureConnected();
      mqttPublishDeviceConnect(dev.c_str());

      bool fire_alert = false;
      bool allKnown = (lastKnown[id].gasAnalog != SENTINEL) &&
                      (lastKnown[id].flameAnalog != SENTINEL) &&
                      (lastKnown[id].temperature != SENTINEL) &&
                      (lastKnown[id].humidity != SENTINEL);
      if (allKnown)
      {
        fire_alert = detectFireFromReadings(lastKnown[id].flameAnalog);
      }
      mqttPublishTelemetry(dev.c_str(),
                           lastKnown[id].gasAnalog,
                           lastKnown[id].flameAnalog,
                           lastKnown[id].temperature,
                           lastKnown[id].humidity,
                           fire_alert,
                           false);
    }
  }

  delay(10);
}
