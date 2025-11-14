#include <WiFi.h>
#include <PubSubClient.h>

// ====== CONFIG WiFi/MQTT ======
const char* WIFI_SSID = "Fercio";
const char* WIFI_PASS = "TW1A2P3D";

const char* MQTT_HOST = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;

// Tópicos
const char* TOPIC_CMD   = "car/cmd";
const char* TOPIC_STATE = "car/state";

// ====== PINES L298N (sin PWM: solo IN1..IN4) ======
const int IN1 = 26;  // Motor A
const int IN2 = 27;
const int IN3 = 14;  // Motor B
const int IN4 = 12;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// --- helpers de movimiento (sin PWM) ---
void motorA_forward()  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
void motorA_backward() { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
void motorA_coast()    { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }  // rueda libre
void motorA_brake()    { digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); }  // freno activo

void motorB_forward()  { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
void motorB_backward() { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
void motorB_coast()    { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
void motorB_brake()    { digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); }

void stopMotors() {
  // elige freno o rueda libre; aquí uso freno activo para parar más rápido
  motorA_brake();
  motorB_brake();
}

void driveForward()  { motorA_forward();  motorB_forward(); }
void driveBackward() { motorA_backward(); motorB_backward(); }
// Giros sobre su eje:
void turnLeft()      { motorA_backward(); motorB_forward(); }
void turnRight()     { motorA_forward();  motorB_backward(); }

void publishState(const char* state) {
  mqtt.publish(TOPIC_STATE, state);
}

// ====== MQTT CALLBACK ======
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == TOPIC_CMD) {
    msg.trim(); msg.toUpperCase();
    if (msg == "F")      { driveForward();  publishState("forward");  }
    else if (msg == "B") { driveBackward(); publishState("backward"); }
    else if (msg == "L") { turnLeft();      publishState("left");     }
    else if (msg == "R") { turnRight();     publishState("right");    }
    else if (msg == "S" || msg == "STOP") { stopMotors(); publishState("stop"); }
    else if (msg == "COAST") { motorA_coast(); motorB_coast(); publishState("coast"); }
    else if (msg == "BRAKE") { stopMotors(); publishState("brake"); }
  }
}

void ensureMqtt() {
  while (!mqtt.connected()) {
    String cid = String("esp32-car-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str())) {
      mqtt.subscribe(TOPIC_CMD);
      publishState("online");
    } else {
      delay(1000);
    }
  }
}

void setup() {
  // GPIO
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors(); // estado seguro al arrancar

  // Serial opcional
  Serial.begin(115200); delay(100);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando WiFi");
  while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(400); }
  Serial.println("\nWiFi OK: " + WiFi.localIP().toString());

  // MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  ensureMqtt();
}

void loop() {
  if (!mqtt.connected()) ensureMqtt();
  mqtt.loop();

  // Latido cada 5 s
  static uint32_t t0 = 0;
  if (millis() - t0 > 5000) {
    t0 = millis();
    mqtt.publish(TOPIC_STATE, "alive");
  }
}