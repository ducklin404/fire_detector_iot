#pragma once
#include <Arduino.h>
#include "types.h"

using EspNowCallback = void(*)(const struct_message &msg);

void espnowInit(EspNowCallback cb);
