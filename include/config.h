#include <Arduino.h>
#pragma once


// WiFi / ThingsBoard
extern const char* WIFI_SSID;
extern const char* WIFI_PASS;
extern const char* THINGSBOARD_SERVER;
extern const uint16_t THINGSBOARD_PORT;
extern const char* THINGSBOARD_GATEWAY_TOKEN;

// Pins & activation logic
#define FAN_PIN 32
#define FAN_ACTIVE_HIGH false
#define ALERT_PIN 33
#define ALERT_ACTIVE_HIGH false



// thresholds & timeouts
extern float FLAME_THRESHOLD;
extern float GAS_THRESHOLD;
extern float HUMID_THRESHOLD;
extern const unsigned long FAN_TIMEOUT_MS;

// misc
extern const int NUM_SENSORS;
extern const float SENTINEL;
extern const unsigned long PERIODIC_SEND_MS;
