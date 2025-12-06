#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define ANA_PIN 34
#define DIG_PIN 27
#define LED_PIN 23 

uint8_t peerMac[] = { 0x84, 0x1F, 0xE8, 0x69, 0x6A, 0xFC };
uint8_t myState     = 0;  // what THIS board is sending (0/1)
uint8_t theirState  = 0;  // last value received from the other board

bool doorState = false;

// Optional: check send status
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// Called when *this* ESP32 receives data from the peer
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len >= 1) {
    theirState = incomingData[0];
    Serial.print("Got theirState = ");
    Serial.println(theirState);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1) Wi-Fi must be in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("This board MAC: ");
  Serial.println(WiFi.macAddress());

  // 2) Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 3) Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // 4) Add peer (the other ESP32)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;      // use current WiFi channel
  peerInfo.encrypt = false;  // no encryption for now

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW two-way node ready.");

  // Improve ADC range
  analogSetPinAttenuation(ANA_PIN, ADC_11db);

  pinMode(LED_PIN, OUTPUT);
  pinMode(DIG_PIN, INPUT);
  pinMode(ANA_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
}

bool lastState = false;

void loop() {
  if (theirState != lastState) {
    lastState = theirState;
    Serial.print("Their state changed to: ");
    Serial.println(theirState);

    // Reflect their state on the LED
    digitalWrite(LED_PIN, theirState ? HIGH : LOW);
  }

  // Example: toggle myState every 2000 ms and send it
  // static unsigned long lastToggle = 0;
  // unsigned long now = millis();

  // if (now - lastToggle >= 2000) {
  //   lastToggle = now;

  //   // Toggle between 0 and 1
  //   myState = (myState == 0) ? 1 : 0;

  //   esp_err_t result = esp_now_send(peerMac, &myState, sizeof(myState));

  //   Serial.print("Sent myState = ");
  //   Serial.print(myState);
  //   Serial.print("  result = ");
  //   Serial.println(result == ESP_OK ? "OK" : "ERROR");
  // }






  // int val   = analogRead(ANA_PIN);
  // int state = digitalRead(DIG_PIN);

  // // Show digital trigger on LED
  // if (state == HIGH) {
  //   doorState = !doorState;
  //   digitalWrite(LED_PIN, doorState ? HIGH : LOW);
  //   delay(500);  // simple debounce
  // }


  // delay(1);
}