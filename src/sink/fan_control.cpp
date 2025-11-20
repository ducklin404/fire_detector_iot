#include "fan_control.h"
#include "config.h"

static volatile unsigned long fanOnUntil = 0;
static bool fanState = false;
static bool autoMode = true;
static bool manualPersistent = false;

static inline void setFanPinPhysical(bool on) {
  if (FAN_ACTIVE_HIGH) digitalWrite(FAN_PIN, on ? HIGH : LOW);
  else digitalWrite(FAN_PIN, on ? LOW : HIGH);
}
static inline void setAlertPinPhysical(bool on) {
  if (ALERT_ACTIVE_HIGH) digitalWrite(ALERT_PIN, on ? HIGH : LOW);
  else digitalWrite(ALERT_PIN, on ? LOW : HIGH);
}

void fanInit() {
  pinMode(FAN_PIN, OUTPUT);
  setFanPinPhysical(false);
  fanState = false;
  fanOnUntil = 0;

  pinMode(ALERT_PIN, OUTPUT);
  setAlertPinPhysical(false);
}


void turnFanOn() {
  setFanPinPhysical(true);
  Serial.printf("[FAN] ON\n");
}

void turnFanOff() {
  setFanPinPhysical(false);
  Serial.printf("[FAN] OFF\n");
}


void turnAlarmOn() {
  setAlertPinPhysical(true);
  Serial.printf("[ALARM] ON\n");
}

void turnAlarmOff() {
  setAlertPinPhysical(false);
  Serial.printf("[ALARM] OFF\n");
}