#include "utils.h"
#include "config.h"

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


float parseFloatToken(const String &token, bool &ok) {
  ok = true;
  String t = token;
  t.trim();
  if (t.length() == 0) {
    ok = false;
    return 0.0f;
  }

  // Strip quotes if the value is a quoted number
  if (t[0] == '\"' && t[t.length() - 1] == '\"') {
    t = t.substring(1, t.length() - 1);
  }

  bool hasDigit = false;
  bool hasDot = false;
  int start = 0;

  // Allow leading +/- sign
  if (t[start] == '+' || t[start] == '-') start++;

  for (int i = start; i < t.length(); i++) {
    char c = t[i];
    if (isdigit(c)) {
      hasDigit = true;
      continue;
    }
    if (c == '.' && !hasDot) {
      hasDot = true;
      continue;
    }
    // Anything else kills the parse
    ok = false;
    return 0.0f;
  }

  if (!hasDigit) {
    ok = false;
    return 0.0f;
  }

  return t.toFloat();
}



static String extractObjectBody(const String &msg, const String &keyName)
{
    int keyPos = msg.indexOf(("\"" + keyName + "\""));
    if (keyPos < 0) return "";
    int bracePos = msg.indexOf('{', keyPos);
    if (bracePos < 0) return "";

    // find matching closing brace with simple brace-counting
    int depth = 0;
    int len = msg.length();
    for (int i = bracePos; i < len; ++i)
    {
        char c = msg.charAt(i);
        if (c == '{') depth++;
        else if (c == '}')
        {
            depth--;
            if (depth == 0)
            {
                return msg.substring(bracePos, i + 1); // include braces
            }
        }
    }
    return "";
}


// apply a single key:value string (value already trimmed / no quotes if any)
static void applyAttributeKeyValue(const String &key, const String &value)
{
    bool ok;
    if (key.equalsIgnoreCase("gas_threshold"))
    {
        float v = parseFloatToken(value, ok);
        if (ok) GAS_THRESHOLD = v;
    }
    else if (key.equalsIgnoreCase("humid_threshold"))
    {
        float v = parseFloatToken(value, ok);
        if (ok) HUMID_THRESHOLD = v;
    }
    else if (key.equalsIgnoreCase("auto_fan"))
    {
        bool v = parseBoolToken(value, ok);
        if (ok)
        {
            auto_fan = v;
            fanMode = auto_fan ? FAN_AUTO : FAN_MANUAL;
        }
    }
    else if (key.equalsIgnoreCase("fan_on"))
    {
        bool v = parseBoolToken(value, ok);
        if (ok) fan_on = v;
    }
    else if (key.equalsIgnoreCase("auto_alarm"))
    {
        bool v = parseBoolToken(value, ok);
        if (ok) auto_alarm = v;
    }
    else if (key.equalsIgnoreCase("alarm_on"))
    {
        bool v = parseBoolToken(value, ok);
        if (ok) alarm_on = v;
    }
    // add more keys as needed
}

// parse simple object payload like {"gas_threshold":400,"auto_fan":true}
static void parseAndApplyObject(const String &obj)
{
    if (obj.length() == 0) return;
    int pos = 0;
    int len = obj.length();
    while (pos < len)
    {
        int q1 = obj.indexOf('\"', pos);
        if (q1 < 0) break;
        int q2 = obj.indexOf('\"', q1 + 1);
        if (q2 < 0) break;
        String key = obj.substring(q1 + 1, q2);
        pos = q2 + 1;

        int colon = obj.indexOf(':', pos);
        if (colon < 0) break;
        pos = colon + 1;
        while (pos < len && isspace(obj.charAt(pos))) pos++;

        String value;
        char c = obj.charAt(pos);
        if (c == '\"')
        {
            int vq = obj.indexOf('\"', pos + 1);
            if (vq < 0) vq = len - 1;
            value = obj.substring(pos + 1, vq);
            pos = vq + 1;
        }
        else
        {
            // read until comma or closing brace
            int comma = obj.indexOf(',', pos);
            int brace = obj.indexOf('}', pos);
            int endPos = -1;
            if (comma < 0 && brace < 0) endPos = len;
            else if (comma < 0) endPos = brace;
            else if (brace < 0) endPos = comma;
            else endPos = min(comma, brace);
            if (endPos < 0) endPos = len;
            value = obj.substring(pos, endPos);
            value.trim();
            pos = endPos;
        }

        applyAttributeKeyValue(key, value);

        // advance past comma
        int nextComma = obj.indexOf(',', pos);
        int nextBrace = obj.indexOf('}', pos);
        if (nextComma >= 0 && (nextComma < nextBrace || nextBrace < 0))
        {
            pos = nextComma + 1;
        }
        else
        {
            break;
        }
    }
}