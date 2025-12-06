#include <Arduino.h>

#define ANA_PIN 34
#define DIG_PIN 27
#define LED_PIN 23 

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIG_PIN, INPUT);
  pinMode(ANA_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
}

bool doorState = false;

void loop() {
  int val   = analogRead(ANA_PIN);
  int state = digitalRead(DIG_PIN);

  // Show digital trigger on LED
  if (state == HIGH) {
    doorState = !doorState;
    digitalWrite(LED_PIN, doorState ? HIGH : LOW);
    delay(500);  // simple debounce
  }


  delay(1);
}
