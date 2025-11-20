#pragma once
#include <Arduino.h>
#include "types.h"

using RpcHandler = String(*)(const String& method, const String& params);

void mqttInit();
void mqttLoop();
void mqttEnsureConnected();
void mqttSetRpcHandler(RpcHandler handler);
void mqttPublishDeviceConnect(const char* deviceName);
void mqttPublishTelemetry(const char* deviceName,
                          float gas, float flame, float temp, float hum,
                          bool fire_alert, bool fan_state);

static void mqttRequestAttributes();