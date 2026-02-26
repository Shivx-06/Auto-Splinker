#include <WiFi.h>
#include <PubSubClient.h>

// ---------------- WiFi ----------------
const char* ssid = "BASS";
const char* password = "987654321";

// ---------------- MQTT ----------------
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

const char* soil_topic = "visawa/soil";
const char* pump_set_topic = "visawa/pump/set";
const char* pump_status_topic = "visawa/pump/status";
const char* auto_topic = "visawa/pump/auto";
const char* auto_status_topic = "visawa/pump/auto/status";

// ---------------- PIN ----------------
#define SOIL_PIN 27
#define RELAY_PIN 26

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- STATE ----------------
bool autoMode = false;

unsigned long autoStartTime = 0;
unsigned long waitStartTime = 0;
unsigned long lastAutoPublish = 0;

enum AutoState {IDLE, CHECK_SOIL, PUMPING, WAITING};
AutoState autoState = IDLE;

// ---------------- WiFi ----------------
void setup_wifi() {
  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");
}

// ---------------- MQTT CALLBACK ----------------
void callback(char* topic, byte* payload, unsigned int length) {

  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  String topicStr = String(topic);

  Serial.println("----- MQTT -----");
  Serial.println("Topic: " + topicStr);
  Serial.println("Message: " + message);

  // ===== MANUAL MODE =====
  if (topicStr == pump_set_topic && !autoMode) {

    if (message == "ON") {
      digitalWrite(RELAY_PIN, HIGH);
      client.publish(pump_status_topic, "ON");
      Serial.println("Pump ON (Manual)");
    }

    if (message == "OFF") {
      digitalWrite(RELAY_PIN, LOW);
      client.publish(pump_status_topic, "OFF");
      Serial.println("Pump OFF (Manual)");
    }
  }

  // ===== AUTO MODE =====
  if (topicStr == auto_topic) {

    if (message == "ON" && !autoMode) {

      autoMode = true;
      autoState = CHECK_SOIL;

      Serial.println("AUTO START");
      client.publish(auto_status_topic, "CHECK:0");
    }

    if (message == "OFF" && autoMode) {

      autoMode = false;
      autoState = IDLE;

      digitalWrite(RELAY_PIN, LOW);

      client.publish(pump_status_topic, "OFF");
      client.publish(auto_status_topic, "STOP");

      Serial.println("AUTO STOP");
    }
  }
}

// ---------------- MQTT RECONNECT ----------------
void reconnect() {

  while (!client.connected()) {

    Serial.println("Connecting MQTT...");

    if (client.connect("ESP32Client123")) {

      Serial.println("MQTT Connected");

      client.subscribe(pump_set_topic);
      client.subscribe(auto_topic);

      client.publish(auto_status_topic, "STOP");  // Sync เว็บ

    } else {

      Serial.println("Retry MQTT...");
      delay(2000);
    }
  }
}

// ---------------- SETUP ----------------
void setup() {

  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SOIL_PIN, INPUT);

  digitalWrite(RELAY_PIN, LOW);

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

// ---------------- LOOP ----------------
void loop() {

  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  // ================= AUTO SYSTEM =================
  if (autoMode) {

    // ----- CHECK SOIL -----
    if (autoState == CHECK_SOIL) {

      int soilState = digitalRead(SOIL_PIN);

      if (soilState == HIGH) {   // DRY

        Serial.println("Soil DRY -> Pump 5 min");

        digitalWrite(RELAY_PIN, HIGH);
        client.publish(pump_status_topic, "ON");

        autoStartTime = now;
        autoState = PUMPING;
      }
      else {   // WET

        Serial.println("Soil WET -> Wait 2 min");

        digitalWrite(RELAY_PIN, LOW);
        client.publish(pump_status_topic, "OFF");

        waitStartTime = now;
        autoState = WAITING;
      }
    }

    // ----- PUMPING -----
    else if (autoState == PUMPING) {

      long remaining = 300 - (now - autoStartTime) / 1000;
      if (remaining < 0) remaining = 0;

      if (now - lastAutoPublish > 1000) {
        lastAutoPublish = now;
        String msg = "PUMPING:" + String(remaining);
        client.publish(auto_status_topic, msg.c_str());
      }

      if (remaining == 0) {

        digitalWrite(RELAY_PIN, LOW);
        client.publish(pump_status_topic, "OFF");

        waitStartTime = now;
        autoState = WAITING;

        Serial.println("Pump finished -> Wait 2 min");
      }
    }

    // ----- WAITING -----
    else if (autoState == WAITING) {

      long remaining = 120 - (now - waitStartTime) / 1000;
      if (remaining < 0) remaining = 0;

      if (now - lastAutoPublish > 1000) {
        lastAutoPublish = now;
        String msg = "WAITING:" + String(remaining);
        client.publish(auto_status_topic, msg.c_str());
      }

      if (remaining == 0) {

        autoState = CHECK_SOIL;
        Serial.println("Wait finished -> Recheck Soil");
      }
    }
  }

  // ================= SOIL PUBLISH =================
  static unsigned long lastSoil = 0;

  if (millis() - lastSoil > 3000) {

    lastSoil = millis();

    String soilStatus = digitalRead(SOIL_PIN) ? "DRY" : "WET";
    client.publish(soil_topic, soilStatus.c_str());
  }
}