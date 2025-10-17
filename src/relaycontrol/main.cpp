#include <Arduino.h>

#define RELAY_PIN_FAN 32  
#define RELAY_PIN_ALERT 33 
#define DELAY_TIME 5000  // 5 seconds in milliseconds

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Relay Toggle Example");

  pinMode(RELAY_PIN_FAN, OUTPUT);
  pinMode(RELAY_PIN_ALERT, OUTPUT);

  // Start with relay OFF
  digitalWrite(RELAY_PIN_ALERT, HIGH);
  digitalWrite(RELAY_PIN_FAN, HIGH);
}

void loop() {
  Serial.println("Alert ON");
  digitalWrite(RELAY_PIN_ALERT, LOW);
  delay(DELAY_TIME);  

  Serial.println("Alert OFF");
  digitalWrite(RELAY_PIN_ALERT, HIGH);
  delay(DELAY_TIME);  

  Serial.println("Fan ON");
  digitalWrite(RELAY_PIN_FAN, LOW);
  delay(DELAY_TIME);

  Serial.println("Fan OFF");
  digitalWrite(RELAY_PIN_FAN, HIGH);
  delay(DELAY_TIME);
}
