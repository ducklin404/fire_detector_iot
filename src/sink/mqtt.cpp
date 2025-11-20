#include "mqtt_client.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "utils.h"
#include "mqtt.h"
#include "fan_control.h"

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static RpcHandler rpcHandler = nullptr;

// local seen devices cache (simple)
#define MAX_DEVICES 24
static String seenDevices[MAX_DEVICES];
static int seenCount = 0;

static bool isSeen(const String &n)
{
    for (int i = 0; i < seenCount; ++i)
        if (seenDevices[i] == n)
            return true;
    return false;
}
static void markSeen(const String &n)
{
    if (seenCount < MAX_DEVICES)
        seenDevices[seenCount++] = n;
}

// helper to send rpc response
static void publishRpcResponse(const String &requestId, const String &responseJson)
{
    if (requestId.length() == 0)
        return;
    String topic = "v1/gateway/rpc/response/" + requestId;
    mqtt.publish(topic.c_str(), responseJson.c_str());
    Serial.printf("[MQTT] RPC response to id=%s -> %s\n", requestId.c_str(), responseJson.c_str());
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    String msg;
    msg.reserve(length + 4);
    for (unsigned int i = 0; i < length; i++)
        msg += (char)payload[i];
    Serial.printf("[MQTT] Message arrived on topic %s: %s\n", topic, msg.c_str());

    String topicStr = String(topic);

    String requestId = extractJsonRawValue(msg, "id");
    requestId.trim();
    if (topicStr == "v1/devices/me/attributes")
    {
        msg.trim();

        // Simple ad-hoc parser for top-level key:value pairs (no nested objects)
        // Works for values: true/false, numbers, or quoted strings.
        int pos = 0;
        int len = msg.length();
        while (pos < len)
        {
            // find next quote for key
            int q1 = msg.indexOf('\"', pos);
            if (q1 < 0)
                break;
            int q2 = msg.indexOf('\"', q1 + 1);
            if (q2 < 0)
                break;
            String key = msg.substring(q1 + 1, q2);
            pos = q2 + 1;

            // find colon
            int colon = msg.indexOf(':', pos);
            if (colon < 0)
                break;
            pos = colon + 1;

            // skip whitespace
            while (pos < len && isspace(msg.charAt(pos)))
                pos++;

            // determine end of value (comma or closing brace)
            int valStart = pos;
            char c = msg.charAt(pos);
            String value;
            if (c == '\"')
            {
                // quoted string
                int vq = msg.indexOf('\"', pos + 1);
                if (vq < 0)
                    vq = len - 1;
                value = msg.substring(pos, vq + 1); // keep quotes for trimming below
                pos = vq + 1;
            }
            else
            {
                // read until comma or closing brace
                int comma = msg.indexOf(',', pos);
                int brace = msg.indexOf('}', pos);
                int endPos = -1;
                if (comma < 0 && brace < 0)
                {
                    endPos = len;
                }
                else if (comma < 0)
                {
                    endPos = brace;
                }
                else if (brace < 0)
                {
                    endPos = comma;
                }
                else
                {
                    endPos = min(comma, brace);
                }
                if (endPos < 0)
                    endPos = len;
                value = msg.substring(pos, endPos);
                pos = endPos;
            }

            // normalize value: trim, strip quotes
            value.trim();
            if (value.startsWith("\"") && value.endsWith("\"") && value.length() >= 2)
            {
                value = value.substring(1, value.length() - 1);
            }
            
            if (rpcHandler){
                String resp = rpcHandler(key, value);
            }

            // move past comma if present
            int nextComma = msg.indexOf(',', pos);
            int nextBrace = msg.indexOf('}', pos);
            if (nextComma >= 0 && (nextComma < nextBrace || nextBrace < 0))
            {
                pos = nextComma + 1;
            }
            else
            {
                break;
            }


        }

        return;
    }
    
}

void mqttInit()
{
    mqtt.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
    mqtt.setCallback(mqttCallback);
}

void mqttEnsureConnected()
{
    if (mqtt.connected())
        return;
    Serial.print("[MQTT] Connecting...");
    String clientId = "esp32_gw_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(clientId.c_str(), THINGSBOARD_GATEWAY_TOKEN, NULL))
    {
        Serial.println(" connected");
        mqtt.subscribe("v1/devices/me/rpc/request/+");
        mqtt.subscribe("v1/devices/me/attributes");
        mqtt.subscribe("v1/devices/me/attributes/response");
        Serial.println("[MQTT] Subscribed to RPC topic");
    }
    else
    {
        Serial.printf(" failed, rc=%d\n", mqtt.state());
    }
}

void mqttLoop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!mqtt.connected())
            mqttEnsureConnected();
        mqtt.loop();
    }
}

void mqttSetRpcHandler(RpcHandler handler)
{
    rpcHandler = handler;
}

void mqttPublishDeviceConnect(const char *deviceName)
{
    String payload = "{\"device\":\"";
    payload += deviceName;
    payload += "\"}";
    mqtt.publish("v1/gateway/connect", payload.c_str());
    markSeen(String(deviceName));
    Serial.printf("[MQTT] connected device %s\n", deviceName);
}

void mqttPublishTelemetry(const char *deviceName,
                          float gas, float flame, float temp, float hum,
                          bool fire_alert, bool fan_state)
{
    String payload = "{ \"";
    payload += deviceName;
    payload += "\":[{";
    bool first = true;
    auto appendKV = [&](const char *k, const String &v)
    {
        if (!first)
            payload += ",";
        payload += "\"";
        payload += k;
        payload += "\":";
        payload += v;
        first = false;
    };

    if (gas != SENTINEL)
        appendKV("gasAnalog", String(gas, 2));
    if (flame != SENTINEL)
        appendKV("flameAnalog", String(flame, 2));
    if (temp != SENTINEL)
        appendKV("temperature", String(temp, 2));
    if (hum != SENTINEL)
        appendKV("humidity", String(hum, 2));

    appendKV("fire_alert", (fire_alert ? "true" : "false"));
    appendKV("fan_state", (fan_state ? "true" : "false"));
    appendKV("auto_mode", (false ? "true" : "false"));
    appendKV("manual_control_allowed", (!false ? "true" : "false"));

    payload += "}]}";

    bool ok = mqtt.publish("v1/gateway/telemetry", payload.c_str());
    Serial.printf("[MQTT] telemetry %s -> %s\n", ok ? "SENT" : "FAILED", deviceName);
    Serial.println(payload);
}


