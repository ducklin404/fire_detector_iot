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



