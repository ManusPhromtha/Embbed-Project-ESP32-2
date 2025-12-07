#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Stepper.h>

#define ANA_PIN 34
#define DIG_PIN 35

#define IR_LED_PIN   33   // KY-005 signal pin

#define MOTOR_IN1   12
#define MOTOR_IN2   14
#define MOTOR_IN3   27
#define MOTOR_IN4   26

const int IR_CHANNEL = 0;
const int IR_FREQ    = 16;
const int IR_RES     = 8;

uint8_t peerMac[] = { 0x84, 0x1F, 0xE8, 0x69, 0x6A, 0xFC };
uint8_t newUnlockState = 0; // last value received from the other board
uint8_t currentUnlockState = 0; // what THIS board is sending (0/1)

// Motor wiring order for ULN2003
const int STEPS_PER_REV = 2048;  // for 28BYJ-48
Stepper stepperMotor(STEPS_PER_REV, MOTOR_IN1, MOTOR_IN3, MOTOR_IN2, MOTOR_IN4);

bool hasSetIRLED = false;

// Optional: check send status
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// Called when *this* ESP32 receives data from the peer
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len >= 1) {
    newUnlockState = incomingData[0];
    Serial.print("Got newUnlockState = ");
    Serial.println(newUnlockState);
  }
}

void lockDoor() {
  newUnlockState = 0;
  currentUnlockState = newUnlockState;

  int quarterTurn = STEPS_PER_REV / 4;
  stepperMotor.step(-quarterTurn);  // choose direction that locks
  delay(2000);
}

void unlockDoor() {
  newUnlockState = 1;
  currentUnlockState = newUnlockState;

  int quarterTurn = STEPS_PER_REV / 4;
  stepperMotor.step(quarterTurn);   // opposite direction
  delay(2000);
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

  // Setup Stepper motor
  stepperMotor.setSpeed(10);  // 10 RPM is safe
  lockDoor();                 // Start locked

  // Setup IR LED
  ledcSetup(IR_CHANNEL, IR_FREQ, IR_RES);
  ledcAttachPin(IR_LED_PIN, IR_CHANNEL);
  ledcWrite(IR_CHANNEL, 0);  // Start with IR LED off

  pinMode(DIG_PIN, INPUT);
  pinMode(ANA_PIN, INPUT);
}

void loop() {
  if (newUnlockState != currentUnlockState && newUnlockState == 0) {
    // Need to lock 1 time
    lockDoor();
    Serial.println("Door locked.");
  } 

  else if (currentUnlockState == 0) {
    // Door is closed, turn off IR LED
    ledcWrite(IR_CHANNEL, 0);
    hasSetIRLED = false;

    int val   = analogRead(ANA_PIN);
    int state = digitalRead(DIG_PIN);

    // Show digital trigger on LED
    if (state == HIGH) {
      unlockDoor();
    }
  } 
  else if (!hasSetIRLED) {
    hasSetIRLED = true;
    ledcWrite(IR_CHANNEL, 128);  // Turn on IR LED at half power

    // Send after activating IR LED
    esp_err_t result = esp_now_send(peerMac, &currentUnlockState, sizeof(currentUnlockState));

    Serial.print("Sent currentUnlockState = ");
    Serial.print(currentUnlockState);
    Serial.print("  result = ");
    Serial.println(result == ESP_OK ? "OK" : "ERROR");
  }
}