#pragma once
#include <Arduino.h>
#include <WString.h>

String extractJsonStringValue(const String &json, const String &key);
String extractJsonRawValue(const String &json, const String &key);
bool parseBoolToken(const String &token, bool &ok);
float parseFloatToken(const String &token, bool &ok);
String extractObjectBody(const String &msg, const String &keyName);
void applyAttributeKeyValue(const String &key, const String &value);
void parseAndApplyObject(const String &obj);