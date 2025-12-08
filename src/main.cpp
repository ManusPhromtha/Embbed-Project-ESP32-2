#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Stepper.h>
#include "DHT.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


#define ANA_PIN 34
#define DIG_PIN 35

#define IR_LED_PIN   33   // KY-005 signal pin

#define MOTOR_IN1   12
#define MOTOR_IN2   14
#define MOTOR_IN3   27
#define MOTOR_IN4   26

#define DHTPIN 4
#define DHTTYPE DHT11

// ==================== CONFIG ====================
const char* ssid         = "TY7373_2.4G";
const char* password     = "0933897373";

const char* mqtt_server  = "mqtt.netpie.io";
const int   mqtt_port    = 1883;
const char* client_id    = "2e551e86-cfa3-41a2-b9b0-cc086ebf6b2c";
const char* device_token = "24R9C4Z1RkuqTU2FumuhCEA6crsZwFaG";
const char* device_secret= "AimrD3qZSbFx9qfGy7nMaNR7zQTrnq85";

// ==================== GLOBAL VARS ====================
WiFiClient   espClient;
PubSubClient client(espClient);
WiFiUDP      ntpUDP;
NTPClient    timeClient(ntpUDP, "pool.ntp.org", 7 * 3600); // GMT+7

// Params for MQTT message
String openTimeStr = "";
String closeTimeStr = "";
float temperature = 0.0;


unsigned long timeStringToSeconds(const String &t) {
  if (t.length() < 8) return 0;  // expect "HH:MM:SS"

  int h = t.substring(0, 2).toInt();
  int m = t.substring(3, 5).toInt();
  int s = t.substring(6, 8).toInt();

  return (unsigned long)h * 3600UL + (unsigned long)m * 60UL + (unsigned long)s;
}

// ==================== WIFI ====================
void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

// ==================== SHADOW UPDATE ====================
void updateShadow() {
  JsonDocument doc;

  // Root: { "data": { ... } }
  JsonObject data   = doc["data"].to<JsonObject>();

  // Group: status
  JsonObject status = data["status"].to<JsonObject>();

  // Group: stat
  JsonObject stat = data["stat"].to<JsonObject>();

  unsigned long openDuration = 0;
  if (openTimeStr.length() > 0 && closeTimeStr.length() > 0) {
    unsigned long openSec  = timeStringToSeconds(openTimeStr);
    unsigned long closeSec = timeStringToSeconds(closeTimeStr);

    if (closeSec >= openSec) {
      openDuration = closeSec - openSec;
    } else {
      // handle wrap-around over midnight
      openDuration = 24UL * 3600UL - openSec + closeSec;
    }
  }

  stat["openDuration"] = openDuration;
  if (!isnan(temperature)) stat["temperature"]   = temperature;

  // Group: time (only if needed)
  if (openTimeStr.length() > 0 || closeTimeStr.length() > 0) {
    JsonObject timeGroup = data["time"].to<JsonObject>();
    if (openTimeStr.length() > 0) {
      timeGroup["lastOpenTime"]  = openTimeStr;
    }
    if (closeTimeStr.length() > 0) {
      timeGroup["lastCloseTime"] = closeTimeStr;
    }
  }

  // Serialize to buffer
  char jsonBuffer[512];
  size_t n = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  if (n == 0) {
    Serial.println("Failed to serialize JSON (buffer too small?)");
    return;
  }

  client.publish("@shadow/data/update", jsonBuffer);
  Serial.print("Sent Update to NETPIE: ");
  Serial.println(jsonBuffer);
}

// ==================== MQTT CONNECT ====================
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to NETPIE...");
    if (client.connect(client_id, device_token, device_secret)) {
      Serial.println(" connected");
      client.subscribe("@shadow/data/delta");

      // Reset dashboard on connect
      updateShadow();
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}


// ==================== MODULE SETUP ====================
DHT dht(DHTPIN, DHTTYPE);

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


void lockDoor() {
  newUnlockState = 0;
  currentUnlockState = newUnlockState;

  closeTimeStr = timeClient.getFormattedTime();
  temperature = dht.readTemperature();
  updateShadow();
  
  int quarterTurn = STEPS_PER_REV / 5;
  stepperMotor.step(-quarterTurn);  // choose direction that locks
  delay(2000);
}

// Optional: check send status
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  if (status != ESP_NOW_SEND_SUCCESS) {
    //resent
    delay(1000);
    esp_err_t result = esp_now_send(peerMac, &currentUnlockState, sizeof(currentUnlockState));
  }
}

// Called when *this* ESP32 receives data from the peer
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len >= 1) {
    newUnlockState = incomingData[0];
    Serial.print("Got newUnlockState = ");
    Serial.println(newUnlockState);
  }
}



void unlockDoor() {
  newUnlockState = 1;
  currentUnlockState = newUnlockState;

  openTimeStr = timeClient.getFormattedTime();
  Serial.println("Door Opened at: " + openTimeStr);

  int quarterTurn = STEPS_PER_REV / 5;
  stepperMotor.step(quarterTurn);   // opposite direction
  delay(2000);
}

// ==================== MQTT CALLBACK ====================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("]: ");

  // Print raw payload
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);

  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  // Check unlockTrigger path: data.status.unlockTrigger
  JsonVariant unlock = doc["data"]["status"]["unlockTrigger"];

  if (!unlock.isNull() && unlock.as<bool>() == true) {
    Serial.println("COMMAND: Unlock Triggered!");
    unlockDoor();
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  // Wi-Fi setup
  WiFi.mode(WIFI_STA);

  connectWiFi();

  timeClient.begin();
  timeClient.update();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // ESP-NOW setup
  Serial.print("This board MAC: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add peer (the other ESP32)
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
  int quarterTurn = STEPS_PER_REV / 5;
  stepperMotor.step(+quarterTurn);
  lockDoor();                 // Start locked

  // Setup IR LED
  ledcSetup(IR_CHANNEL, IR_FREQ, IR_RES);
  ledcAttachPin(IR_LED_PIN, IR_CHANNEL);
  ledcWrite(IR_CHANNEL, 0);  // Start with IR LED off

  dht.begin();
  
  pinMode(DIG_PIN, INPUT);
  pinMode(ANA_PIN, INPUT);
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
  timeClient.update();

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