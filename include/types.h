#pragma once
#include <Arduino.h>

typedef struct struct_message {
  int id;
  float gasAnalog;
  float flameAnalog;
  float temperature;
  float humidity;
} struct_message;
