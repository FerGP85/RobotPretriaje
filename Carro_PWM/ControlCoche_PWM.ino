#include <WiFi.h>
#include <PubSubClient.h>

/* ====== CONFIG WiFi/MQTT ====== */
const char* WIFI_SSID = "Fercio";
const char* WIFI_PASS = "TW1A2P3D";

const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

/* Tópicos */
const char* TOPIC_CMD   = "car/cmd";
const char* TOPIC_STATE = "car/state";

/* ====== PINES L298N (con PWM) ====== */
// Motor A
const int ENA = 18;
const int IN1 = 22;
const int IN2 = 21;
// Motor B
const int ENB = 19;
const int IN3 = 17;
const int IN4 = 16;

/* ====== PWM ====== */
// Usamos analogWrite() para compatibilidad inmediata.
// (Si luego migras a LEDC, se puede portar sin cambiar la lógica.)
int duty = 160;            // 0..255 velocidad por defecto
const int KICK_DUTY = 255; // golpe inicial opcional
const int KICK_MS   = 120; // duración del kick
bool useKick = false;       // si quieres, pon false para quitar el golpe

WiFiClient espClient;
PubSubClient mqtt(espClient);

/* ====== Helpers de dirección ====== */
void setDirA(int v){ // v>0 fwd, v<0 back, v=0 free
  if (v > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (v < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else            { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
}
void setDirB(int v){
  if (v > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else if (v < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else            { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
}

/* ====== Helpers PWM ====== */
void setPWM_A(int d){ if (d<0) d=0; if (d>255) d=255; analogWrite(ENA, d); }
void setPWM_B(int d){ if (d<0) d=0; if (d>255) d=255; analogWrite(ENB, d); }

/* ====== Movimientos ====== */
void stopMotors() {
  // freno activo + duty 0
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  setPWM_A(0); setPWM_B(0);
}

void applyKickIfNeeded(int d){
  if (useKick && d > 0) {
    setPWM_A(KICK_DUTY); setPWM_B(KICK_DUTY);
    delay(KICK_MS);
  }
}

void driveForward()  {
  setDirA(+1); setDirB(+1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}
void driveBackward() {
  setDirA(-1); setDirB(-1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}
// Giros sobre su eje:
void turnLeft()  {
  setDirA(-1); setDirB(+1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}
void turnRight() {
  setDirA(+1); setDirB(-1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}

void publishState(const char* state) {
  mqtt.publish(TOPIC_STATE, state);
}

/* ====== MQTT CALLBACK ====== */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if (String(topic) == TOPIC_CMD) {
    String up = msg; up.toUpperCase();

    // Comandos básicos
    if (up == "F")           { driveForward();  publishState("forward");  return; }
    if (up == "B")           { driveBackward(); publishState("backward"); return; }
    if (up == "L")           { turnLeft();      publishState("left");     return; }
    if (up == "R")           { turnRight();     publishState("right");    return; }
    if (up == "S" || up=="STOP") { stopMotors();  publishState("stop");     return; }
    if (up == "COAST") { // rueda libre (entradas en LOW y PWM 0)
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
      setPWM_A(0); setPWM_B(0);
      publishState("coast"); return;
    }
    if (up == "BRAKE") { stopMotors(); publishState("brake"); return; }

    // Ajuste de velocidad: SPD:0..255 (ej. "SPD:200")
    // Acepta minúsculas o mayúsculas.
    if (up.startsWith("SPD:")) {
      int val = msg.substring(4).toInt();
      if (val < 0) val = 0; if (val > 255) val = 255;
      duty = val;
      publishState((String("speed=") + String(duty)).c_str());
      return;
    }
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

  // Asegura PWM en 0 al iniciar
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  stopMotors(); // estado seguro

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

  publishState("ready");
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