/****************************************
 * Includes
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_MLX90614.h>  // MLX90614 (Wire)
#include "MAX30105.h"           // MAX30102 (SparkFun)
#include "spo2_algorithm.h"     // Define FreqS=25 y BUFFER_SIZE=100

// ===== ESP32 BLE Arduino (nkolban) =====
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

/****************************************
 * Wi-Fi
 ****************************************/
const char* WIFI_SSID = "Fercio";
const char* WIFI_PASS = "TW1A2P3D";

/****************************************
 * MQTT
 ****************************************/
const char* MQTT_SERVER    = "broker.hivemq.com";
const int   MQTT_PORT      = 1883;
const char* MQTT_CLIENT_ID = "r2d2";
const char* MQTT_USER      = "r2d2";
const char* MQTT_PASS_MQTT = "r2d2";

WiFiClient   espClient;
PubSubClient client(espClient);

/****************************************
 * Topics MQTT
 ****************************************/
// Carro
const char* TOPIC_CMD   = "car/cmd";
const char* TOPIC_STATE = "car/state";

// Salud
const char* TOPIC_HEALTH  = "esp32/health";
const char* TOPIC_HR      = "esp32/health/hr";
const char* TOPIC_SPO2    = "esp32/health/spo2";
const char* TOPIC_TA      = "esp32/health/ta";
const char* TOPIC_TO      = "esp32/health/to";
const char* TOPIC_SCALE   = "esp32/scale/weight_kg";

/****************************************
 * Pines del carro (L298N con PWM)
 ****************************************/
// Motor A
const int ENA = 18;
const int IN1 = 27;
const int IN2 = 26;
// Motor B
const int ENB = 19;
const int IN3 = 17;
const int IN4 = 16;

/****************************************
 * Sensor ultrasónico anti-caída (HC-SR04)
 ****************************************/
const int US_TRIG = 4;
const int US_ECHO = 5;

const float CLIFF_THRESHOLD_CM = 6.0f;
const uint32_t BACK_TIME_MS = 600;
const uint32_t TURN_TIME_MS = 700;

enum class CliffState : uint8_t { NORMAL, BACKING, TURNING };
CliffState cliffState = CliffState::NORMAL;
bool cliffActive = false;
uint32_t cliffStateStart = 0;

/* ====== PWM ====== */
int duty = 160;
const int KICK_DUTY = 255;
const int KICK_MS   = 120;
bool useKick = false;

uint32_t lastCarCmdMs = 0;

/****************************************
 * I²C único (Wire)
 ****************************************/
#define I2C_SDA  21
#define I2C_SCL  22
#define I2C_FREQ 100000

/****************************************
 * Sensores
 ****************************************/
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
bool mlxOK = false;

MAX30105 particleSensor;
bool maxOK = false;

// Del algoritmo MAX: FreqS=25 => 40 ms
#define SAMPLE_DELAY_MS (1000 / FreqS)

/****************************************
 * MLX offset
 ****************************************/
const float MLX_OFFSET_C = 2.5f;

/****************************************
 * Filtro simple HR/SpO2 (EMA + outlier reject)
 ****************************************/
float hr_f = -1.0f, spo2_f = -1.0f;
const float ALPHA_HR = 0.20f;
const float ALPHA_SPO2 = 0.20f;

inline bool acceptHR(int32_t hr)   { return hr >= 40 && hr <= 180; }
inline bool acceptSpO2(int32_t s2) { return s2 >= 85 && s2 <= 100; }

inline void updateFilters(int32_t hr, int8_t hrValid, int32_t s2, int8_t s2Valid) {
  if (hrValid && acceptHR(hr)) {
    if (hr_f < 0) hr_f = hr;
    else if (fabsf(hr - hr_f) <= 30.0f)
      hr_f = (1.0f - ALPHA_HR) * hr_f + ALPHA_HR * hr;
  }
  if (s2Valid && acceptSpO2(s2)) {
    if (spo2_f < 0) spo2_f = s2;
    else if (fabsf(s2 - spo2_f) <= 4.0f)
      spo2_f = (1.0f - ALPHA_SPO2) * spo2_f + ALPHA_SPO2 * s2;
  }
}
inline void resetFilters() { hr_f = -1.0f; spo2_f = -1.0f; }

/****************************************
 * Calibración HR/SpO2
 ****************************************/
float CAL_HR_BPM   = -10.0f;
float CAL_SPO2_PCT = -5.0f;
const int SPO2_MIN = 88, SPO2_MAX = 100;

/****************************************
 * AGC para MAX30102 (igual que tu funcional)
 ****************************************/
const uint32_t IR_TARGET  = 60000;
const uint32_t RED_TARGET = 45000;
const uint32_t DC_TOL     = 8000;

const uint8_t LED_MIN = 0x08;
const uint8_t LED_MAX = 0x60;

uint8_t ledIR  = 0x1F;
uint8_t ledRED = 0x1F;

/****************************************
 * Peak-based BPM (IR) (igual que tu funcional)
 ****************************************/
const float HR_MIN = 40.0f, HR_MAX = 180.0f;
const float HP_ALPHA = 0.95f;
const float LP_ALPHA = 0.20f;
const uint32_t REFRACT_MS = 350;
const float TH_K = 0.7f;

float ir_dc = 0.0f;
float ir_env = 0.0f;
uint32_t lastPeakMs = 0;
float lastPPMs[5] = {0};
uint8_t nPP = 0;
float g_bpm_peak = NAN;

static bool processIRSampleForPeak(uint32_t ir, uint32_t nowMs, float &bpmOut) {
  ir_dc = HP_ALPHA * ir_dc + (1.0f - HP_ALPHA) * (float)ir;
  float x = (float)ir - ir_dc;

  float ax = fabsf(x);
  ir_env = (1.0f - LP_ALPHA) * ir_env + LP_ALPHA * ax;

  float thr = TH_K * ir_env;

  static bool wasAbove = false;
  bool isAbove = (x > thr);

  bool peak = false;
  if (isAbove && !wasAbove) {
    if (lastPeakMs == 0 || (nowMs - lastPeakMs) >= REFRACT_MS) {
      if (lastPeakMs != 0) {
        float ppm = (float)(nowMs - lastPeakMs);
        if (ppm > 300.0f && ppm < 1500.0f) {
          if (nPP < 5) { lastPPMs[nPP++] = ppm; }
          else {
            for (int i=1;i<5;i++) lastPPMs[i-1] = lastPPMs[i];
            lastPPMs[4] = ppm;
          }
          float sum = 0; for (int i=0;i<nPP;i++) sum += lastPPMs[i];
          float meanPP = sum / (float)nPP;
          float bpm = 60000.0f / meanPP;
          if (bpm >= HR_MIN && bpm <= HR_MAX) {
            bpmOut = bpm;
            peak = true;
          }
        }
      }
      lastPeakMs = nowMs;
    }
  }
  wasAbove = isAbove;
  return peak;
}

/****************************************
 * Tiempo / timers
 ****************************************/
uint32_t lastUpdateMs = 0;
const uint32_t UPDATE_MS = 2000;

uint32_t lastRSSI = 0;

// Peso caching
float     lastWeightKg = NAN;
uint32_t  lastWeightAt = 0;
const uint32_t WEIGHT_FRESH_MS = 8000;

// "alive" del carro
uint32_t lastCarAlive = 0;
const uint32_t CAR_ALIVE_MS = 5000;

/****************************************
 * Wi-Fi robusto (state machine)
 ****************************************/
enum class WifiState : uint8_t { IDLE, CONNECTING, CONNECTED };
volatile WifiState wifiState = WifiState::IDLE;

uint8_t targetBSSID[6] = {0};
int     targetChannel  = 0;
bool    useBSSIDLock   = true;

const char* reasonText(uint8_t r){
  switch(r){
    case 1:  return "UNSPECIFIED";
    case 2:  return "AUTH_EXPIRE";
    case 3:  return "AUTH_LEAVE";
    case 4:  return "ASSOC_EXPIRE";
    case 5:  return "ASSOC_TOOMANY";
    case 6:  return "NOT_AUTHED";
    case 7:  return "NOT_ASSOCED";
    case 8:  return "ASSOC_LEAVE";
    case 9:  return "ASSOC_NOT_AUTHED";
    case 15: return "4WAY_HANDSHAKE_TIMEOUT";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    default: return "UNKNOWN";
  }
}

void wifiFindAP(const char* ssid) {
  targetChannel = 0;
  memset(targetBSSID, 0, sizeof(targetBSSID));
  Serial.println("[WiFi] Escaneando AP objetivo...");
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) { Serial.println("[WiFi] No se encontraron redes"); return; }
  int best = -1000; int idxSel = -1;
  for (int i=0;i<n;i++){
    if (WiFi.SSID(i) == ssid) {
      int rssi = WiFi.RSSI(i);
      if (rssi > best) { best = rssi; idxSel = i; }
    }
  }
  if (idxSel >= 0) {
    targetChannel = WiFi.channel(idxSel);
    auto b = WiFi.BSSID(idxSel);
    memcpy(targetBSSID, b, 6);
    Serial.printf("[WiFi] AP '%s' encontrado. Canal=%d RSSI=%d dBm BSSID=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  ssid, targetChannel, best,
                  targetBSSID[0],targetBSSID[1],targetBSSID[2],
                  targetBSSID[3],targetBSSID[4],targetBSSID[5]);
  } else {
    Serial.println("[WiFi] No se encontró el SSID objetivo.");
  }
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  switch(event){
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi] STA_START");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiState = WifiState::CONNECTED;
      Serial.printf("[WiFi] GOT_IP: %s (RSSI %d dBm)\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      wifiState = WifiState::IDLE;
      uint8_t r = info.wifi_sta_disconnected.reason;
      Serial.printf("[WiFi] DISCONNECTED reason=%u (%s)\n", r, reasonText(r));
      if (r == 15 || r == 202 || r == 204) useBSSIDLock = false;
      break;
    }
    default: break;
  }
}

void wifiInitStation(){
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.onEvent(onWiFiEvent);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void wifiStartConnect(){
  if (wifiState == WifiState::CONNECTING) return;

  WiFi.disconnect(true, true);
  delay(50);

  if (useBSSIDLock && targetChannel > 0 && targetBSSID[0] != 0) {
    Serial.printf("[WiFi] Conectando a \"%s\" anclado...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS, targetChannel, targetBSSID, true);
  } else {
    Serial.printf("[WiFi] Conectando a \"%s\"...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  wifiState = WifiState::CONNECTING;
}

void wifiService(){
  static uint32_t nextAction = 0;
  static uint16_t backoffMs  = 1000;
  static uint32_t startedAt  = 0;

  if (millis() < nextAction) return;

  if (wifiState == WifiState::CONNECTED) {
    backoffMs = 1000;
    startedAt = 0;
    return;
  }

  if (wifiState == WifiState::CONNECTING) {
    if (startedAt == 0) startedAt = millis();
    if (millis() - startedAt > 8000) {
      Serial.println("[WiFi] Timeout conectando. Reintento...");
      wifiFindAP(WIFI_SSID);
      useBSSIDLock = false;
      wifiState = WifiState::IDLE;
      startedAt = 0;
      backoffMs = min<uint16_t>(backoffMs * 2, 30000);
      nextAction = millis() + backoffMs;
    } else {
      nextAction = millis() + 300;
    }
    return;
  }

  if (useBSSIDLock && (targetChannel == 0 || targetBSSID[0] == 0)) wifiFindAP(WIFI_SSID);
  wifiStartConnect();

  nextAction = millis() + backoffMs;
  backoffMs = min<uint16_t>(backoffMs * 2, 30000);
}

/****************************************
 * MQTT helpers
 ****************************************/
void mqttSetup(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);
  client.setBufferSize(256);
}

void mqttService(){
  static uint32_t nextTry = 0;
  static uint16_t backoff = 1000;

  if (wifiState != WifiState::CONNECTED) {
    nextTry = millis() + 1000;
    backoff = 1000;
    return;
  }

  if (client.connected()) {
    client.loop();
    backoff = 1000;
    return;
  }

  if (millis() < nextTry) return;

  Serial.print("[MQTT] Conectando...");
  String clientId = String(MQTT_CLIENT_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS_MQTT);

  if (ok) {
    Serial.println("OK");
    backoff = 1000;
    client.subscribe(TOPIC_CMD);
    client.publish(TOPIC_STATE, "online");
  } else {
    Serial.printf("falló rc=%d\n", client.state());
    backoff = backoff < 30000 ? backoff * 2 : 30000;
  }
  nextTry = millis() + backoff;
}

/****************************************
 * Background pump (para NO matar MQTT/WiFi mientras MAX bloquea)
 ****************************************/
float readCliffDistanceCm(); // forward
void cliffService();         // forward

static inline void backgroundPump() {
  cliffService();
  wifiService();
  mqttService();
  if (client.connected()) client.loop();
  yield();
}

/****************************************
 * BLE Mi Body Scale
 ****************************************/
static const char* UUID_181B = "0000181b-0000-1000-8000-00805f9b34fb";
static const char* UUID_181D = "0000181d-0000-1000-8000-00805f9b34fb";

static const float EXPECTED_MIN_KG = 30.0f;
static const float EXPECTED_MAX_KG = 90.0f;

static inline bool inRange(float v, float lo, float hi) { return !isnan(v) && v >= lo && v <= hi; }

static float readU16Scaled(const uint8_t* d, int len, int off, float scale) {
  if (off + 1 >= len) return NAN;
  uint16_t raw = (uint16_t)d[off] | ((uint16_t)d[off + 1] << 8);
  float kg = raw / scale;
  return kg > 0.f && kg < 300.f ? kg : NAN;
}

static float pickWeightFrom181x(const uint8_t* sd, int sl) {
  float kg_11_200 = readU16Scaled(sd, sl, 11, 200.0f);
  if (inRange(kg_11_200, EXPECTED_MIN_KG, EXPECTED_MAX_KG)) return kg_11_200;

  float kg_11_100 = readU16Scaled(sd, sl, 11, 100.0f);
  if (inRange(kg_11_100, EXPECTED_MIN_KG, EXPECTED_MAX_KG)) return kg_11_100;

  float kg_00_100 = readU16Scaled(sd, sl, 0, 100.0f);
  if (inRange(kg_00_100, EXPECTED_MIN_KG, EXPECTED_MAX_KG)) return kg_00_100;

  if (!isnan(kg_11_200)) return kg_11_200;
  if (!isnan(kg_11_100)) return kg_11_100;
  if (!isnan(kg_00_100)) return kg_00_100;
  return NAN;
}

static void publishWeight(float kg) {
  Serial.printf("[scale] %.2f kg\n", kg);
  lastWeightKg = kg;
  lastWeightAt = millis();
  if (client.connected()) {
    char buf[24];
    dtostrf(kg, 0, 2, buf);
    client.publish(TOPIC_SCALE, buf, true);
  }
}

class AdvCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    int cnt = dev.getServiceDataCount();
    if (cnt <= 0) return;

    for (int i = 0; i < cnt; ++i) {
      String uuid = String(dev.getServiceDataUUID(i).toString().c_str());
      if (!uuid.equalsIgnoreCase(UUID_181B) && !uuid.equalsIgnoreCase(UUID_181D)) continue;

      String data = dev.getServiceData(i);
      const uint8_t* sd = (const uint8_t*)data.c_str();
      int sl = data.length();

      float kg = pickWeightFrom181x(sd, sl);
      if (!isnan(kg)) publishWeight(kg);
    }
  }
};

BLEScan* g_bleScan = nullptr;

void scaleInit() {
  BLEDevice::init("");
  g_bleScan = BLEDevice::getScan();
  g_bleScan->setAdvertisedDeviceCallbacks(new AdvCallbacks(), true);
  g_bleScan->setActiveScan(true);
  g_bleScan->setInterval(80);
  g_bleScan->setWindow(70);
}

void scaleService() {
  if (!g_bleScan) return;

  static uint32_t lastScan = 0;
  uint32_t now = millis();

  const uint32_t SCAN_PERIOD_MS        = 5000;
  const uint32_t NO_SCAN_AFTER_CMD_MS  = 2000;

  if (now - lastCarCmdMs < NO_SCAN_AFTER_CMD_MS) return;
  if (now - lastScan < SCAN_PERIOD_MS) return;
  lastScan = now;

  g_bleScan->start(1, false);
  g_bleScan->clearResults();
}

/****************************************
 * Carro helpers
 ****************************************/
void setDirA(int v){
  if (v > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (v < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else            { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
}
void setDirB(int v){
  if (v > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else if (v < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else            { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
}

void setPWM_A(int d){ d = constrain(d, 0, 255); analogWrite(ENA, d); }
void setPWM_B(int d){ d = constrain(d, 0, 255); analogWrite(ENB, d); }

void stopMotors() {
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

void driveForward()  { setDirA(-1); setDirB(-1); applyKickIfNeeded(duty); setPWM_A(duty); setPWM_B(duty); }
void driveBackward() { setDirA(+1); setDirB(+1); applyKickIfNeeded(duty); setPWM_A(duty); setPWM_B(duty); }
void turnLeft()      { setDirA(+1); setDirB(-1); applyKickIfNeeded(duty); setPWM_A(duty); setPWM_B(duty); }
void turnRight()     { setDirA(-1); setDirB(+1); applyKickIfNeeded(duty); setPWM_A(duty); setPWM_B(duty); }

void publishCarState(const char* state) {
  if (client.connected()) client.publish(TOPIC_STATE, state);
}

/****************************************
 * MQTT comando carro
 ****************************************/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_CMD) != 0) return;

  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String up = msg; up.toUpperCase();

  if (cliffActive) {
    if (up == "S" || up == "STOP" || up == "BRAKE") {
      stopMotors();
      publishCarState("user_stop_during_cliff");
      lastCarCmdMs = millis();
    } else {
      publishCarState("cliff_lockout");
    }
    return;
  }

  if (up == "F") { driveForward();  publishCarState("forward");  lastCarCmdMs = millis(); return; }
  if (up == "B") { driveBackward(); publishCarState("backward"); lastCarCmdMs = millis(); return; }
  if (up == "L") { turnLeft();      publishCarState("left");     lastCarCmdMs = millis(); return; }
  if (up == "R") { turnRight();     publishCarState("right");    lastCarCmdMs = millis(); return; }

  if (up == "S" || up == "STOP") { stopMotors(); publishCarState("stop"); lastCarCmdMs = millis(); return; }

  if (up == "COAST") {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    setPWM_A(0); setPWM_B(0);
    publishCarState("coast");
    lastCarCmdMs = millis();
    return;
  }

  if (up == "BRAKE") { stopMotors(); publishCarState("brake"); lastCarCmdMs = millis(); return; }

  if (up.startsWith("SPD:")) {
    int val = msg.substring(4).toInt();
    duty = constrain(val, 0, 255);
    String s = String("speed=") + String(duty);
    publishCarState(s.c_str());
    lastCarCmdMs = millis();
    return;
  }
}

/****************************************
 * Lectura ultrasónico / cliff
 ****************************************/
float readCliffDistanceCm() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(US_ECHO, HIGH, 15000UL);
  if (duration == 0) return 999.0f;
  return (duration * 0.0343f) / 2.0f;
}

// ✅ ESTA FUNCIÓN YA VA CON TODAS LAS LLAVES BIEN
void cliffService() {
  static uint32_t lastMeasure = 0;
  uint32_t now = millis();

  if (cliffState == CliffState::BACKING) {
    if (now - cliffStateStart >= BACK_TIME_MS) {
      stopMotors();
      cliffState = CliffState::TURNING;
      cliffStateStart = now;
      turnRight();
      publishCarState("cliff_turn");
    }
    return;
  }

  if (cliffState == CliffState::TURNING) {
    if (now - cliffStateStart >= TURN_TIME_MS) {
      stopMotors();
      cliffState = CliffState::NORMAL;
      cliffActive = false;
      publishCarState("cliff_done");
    }
    return;
  }

  if (now - lastMeasure < 100) return;
  lastMeasure = now;

  float d = readCliffDistanceCm();

  if (d < CLIFF_THRESHOLD_CM) {
    Serial.printf("[cliff] POSIBLE CAÍDA d=%.1f cm\n", d);

    cliffActive = true;
    cliffState  = CliffState::BACKING;
    cliffStateStart = now;

    stopMotors();
    delay(20);
    driveBackward();
    publishCarState("cliff_back");
    lastCarCmdMs = millis();
  }
}

/****************************************
 * Sensores: init MAX/MLX
 ****************************************/
void scanI2C(TwoWire &bus, const char* name) {
  Serial.printf("\n[SCAN] %s\n", name);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.printf("  - Dispositivo en 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  (sin dispositivos)");
}

bool initMLX() {
  for (int i = 0; i < 3; i++) {
    if (mlx.begin(0x5A, &Wire)) return true;
    delay(50);
  }
  return false;
}

// SampleRate=100; pero pacing a 25 Hz al llenar buffer
bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) return false;

  byte sampleAverage = 4;
  byte ledMode = 2;
  int  sampleRate   = 100;
  int  pulseWidth   = 411;
  int  adcRange     = 16384;

  ledRED = 0x1F;
  ledIR  = 0x1F;

  particleSensor.setup(ledIR, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(ledRED);
  particleSensor.setPulseAmplitudeIR(ledIR);
  particleSensor.setPulseAmplitudeGreen(0);

  particleSensor.clearFIFO();
  return true;
}

bool readTemps(double &Ta, double &To) {
  if (!mlxOK) { Ta = NAN; To = NAN; return false; }
  Ta = mlx.readAmbientTempC();
  To = mlx.readObjectTempC();
  To += MLX_OFFSET_C;
  return isfinite(Ta) && isfinite(To);
}

/****************************************
 * Finger detect EXACTO (tu funcional)
 ****************************************/
bool fingerPresent() {
  if (!maxOK) return false;

  const int N = 16;
  uint32_t ir;
  double sum = 0.0, sum2 = 0.0;

  for (int i = 0; i < N; i++) {
    uint32_t t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      backgroundPump();
      if (millis() - t0 > 200) break;
      delay(1);
    }
    if (!particleSensor.available()) continue;

    ir = particleSensor.getIR();
    particleSensor.nextSample();
    sum  += ir;
    sum2 += (double)ir * (double)ir;
  }

  double mean = sum / (double)N;
  double var  = (sum2 / (double)N) - (mean * mean);
  double stdv = (var > 0) ? sqrt(var) : 0;

  const double MEAN_MIN = 20000.0;
  const double STD_MIN  = 20.0;

  return (mean >= MEAN_MIN) && (stdv >= STD_MIN);
}

/****************************************
 * Quick DC + AGC (tu funcional)
 ****************************************/
static bool readQuickDC(uint32_t &irDC, uint32_t &redDC, int N = 32, uint32_t timeout_ms = 500) {
  if (!maxOK) return false;
  uint64_t sir=0, srd=0;
  int got=0;
  uint32_t t0 = millis();
  while (got < N) {
    while (!particleSensor.available()) {
      particleSensor.check();
      backgroundPump();
      if (millis() - t0 > timeout_ms) break;
      delay(1);
    }
    if (!particleSensor.available()) break;
    uint32_t red = particleSensor.getRed();
    uint32_t ir  = particleSensor.getIR();
    particleSensor.nextSample();
    sir += ir; srd += red; got++;
  }
  if (got < N/2) return false;
  irDC = (uint32_t)(sir / got);
  redDC= (uint32_t)(srd / got);
  return true;
}

static void agcAdjust() {
  uint32_t irDC=0, redDC=0;
  if (!readQuickDC(irDC, redDC)) return;

  if (irDC < IR_TARGET - DC_TOL && ledIR < LED_MAX) {
    ledIR = min<uint8_t>(LED_MAX, (uint8_t)(ledIR + 4));
    particleSensor.setPulseAmplitudeIR(ledIR);
  } else if (irDC > IR_TARGET + DC_TOL && ledIR > LED_MIN) {
    ledIR = max<uint8_t>(LED_MIN, (uint8_t)(ledIR - 4));
    particleSensor.setPulseAmplitudeIR(ledIR);
  }

  if (redDC < RED_TARGET - DC_TOL && ledRED < LED_MAX) {
    ledRED = min<uint8_t>(LED_MAX, (uint8_t)(ledRED + 4));
    particleSensor.setPulseAmplitudeRed(ledRED);
  } else if (redDC > RED_TARGET + DC_TOL && ledRED > LED_MIN) {
    ledRED = max<uint8_t>(LED_MIN, (uint8_t)(ledRED - 4));
    particleSensor.setPulseAmplitudeRed(ledRED);
  }
}

/****************************************
 * readHRSpO2 EXACTO (tu funcional) + pump adentro
 ****************************************/
bool readHRSpO2(int32_t &hr, int8_t &hrValid, int32_t &spo2, int8_t &spo2Valid) {
  if (!maxOK) { hr=-1; hrValid=0; spo2=-1; spo2Valid=0; return false; }

  uint32_t irBuffer[BUFFER_SIZE];
  uint32_t redBuffer[BUFFER_SIZE];

  for (int i = 0; i < BUFFER_SIZE; i++) {
    uint32_t t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      backgroundPump();
      if (millis() - t0 > 2000) return false;
      delay(1);
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();

    uint32_t t1 = millis();
    while (millis() - t1 < SAMPLE_DELAY_MS) {
      particleSensor.check();
      backgroundPump();
      delay(1);
    }
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2, &spo2Valid, &hr, &hrValid);

  g_bpm_peak = NAN;
  {
    uint32_t now = millis();
    for (int i = 0; i < BUFFER_SIZE; i++) {
      float tmp;
      bool got = processIRSampleForPeak(irBuffer[i], now + i * SAMPLE_DELAY_MS, tmp);
      if (got) g_bpm_peak = tmp;
    }
  }

  return (hrValid == 1 || spo2Valid == 1);
}

/****************************************
 * Setup
 ****************************************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[START] ESP32 Car + Health + BLE Scale + WiFi/MQTT (MAX modo funcional)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  scanI2C(Wire, "Wire (SDA=21, SCL=22)");

  mlxOK = initMLX();
  Serial.println(mlxOK ? "MLX90614 OK (0x5A)" : "MLX90614 NO detectado");

  maxOK = initMAX30102();
  Serial.println(maxOK ? "MAX30102 OK (0x57 tipico)" : "MAX30102 NO detectado");

  // Motores
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  stopMotors();

  // Ultrasonico
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  // WiFi
  wifiInitStation();
  wifiFindAP(WIFI_SSID);
  wifiStartConnect();

  // MQTT
  mqttSetup();
  client.setCallback(mqttCallback);

  // BLE scale
  scaleInit();
}

/****************************************
 * Loop
 ****************************************/
void loop() {
  // Seguridad primero
  cliffService();

  // Red / MQTT
  wifiService();
  mqttService();

  // BLE báscula
  scaleService();

  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_MS) {
    lastUpdateMs = now;

    // Temperaturas
    double Ta = NAN, To = NAN;
    bool tempOK = readTemps(Ta, To);

    // HR/SpO2 EXACTO al funcional:
    bool hasFinger = fingerPresent();

    int32_t hr = -1; int8_t hrValid = 0;
    int32_t s2 = -1; int8_t s2Valid = 0;

    if (hasFinger) {
      (void)readHRSpO2(hr, hrValid, s2, s2Valid);
      updateFilters(hr, hrValid, s2, s2Valid);
      agcAdjust();
    } else {
      resetFilters();
      g_bpm_peak = NAN;
    }

    // Selección HR: igual que tu funcional
    float hr_sel = -1.0f;
    int32_t hr_alg = hasFinger ? ((hr_f > 0) ? (int32_t)lroundf(hr_f) : hr) : -1;

    bool peak_ok = (!isnan(g_bpm_peak) && g_bpm_peak >= HR_MIN && g_bpm_peak <= HR_MAX);

    if (peak_ok && hr_alg > 0) hr_sel = 0.7f * g_bpm_peak + 0.3f * (float)hr_alg;
    else if (peak_ok)          hr_sel = g_bpm_peak;
    else if (hr_alg > 0)       hr_sel = (float)hr_alg;

    int32_t hr_show = -1;
    if (hasFinger && hr_sel > 0) {
      hr_show = (int32_t)lroundf(hr_sel + CAL_HR_BPM);
      hr_show = constrain(hr_show, 40, 200);
    }

    // SpO2 igual que tu funcional
    int32_t s2_raw_show = hasFinger ? ((spo2_f > 0) ? (int32_t)lroundf(spo2_f) : s2) : -1;
    int32_t s2_show = s2_raw_show;
    if (hasFinger && s2_show > 0) {
      s2_show = (int32_t)lroundf((float)s2_show + CAL_SPO2_PCT);
      s2_show = constrain(s2_show, SPO2_MIN, SPO2_MAX);
    } else {
      s2_show = -1;
    }

    float weight_for_json = (!isnan(lastWeightKg) && (now - lastWeightAt <= WEIGHT_FRESH_MS))
                              ? lastWeightKg : -1.0f;

    // Serial
    Serial.print("Ta=");
    if (tempOK) Serial.print(Ta, 1); else Serial.print("NaN");
    Serial.print("C | To=");
    if (tempOK) Serial.print(To, 1); else Serial.print("NaN");
    Serial.print("C  ||  HR=");
    Serial.print(hr_show);
    Serial.print(" bpm | SpO2=");
    Serial.print(s2_show);
    Serial.print("% | Weight=");
    Serial.print(weight_for_json, 2);
    Serial.println(" kg");

    // MQTT (compat)
    if (client.connected()) {
      char json[260];
      snprintf(json, sizeof(json),
               "{\"hr\":%ld,\"hrValid\":%d,\"spo2\":%ld,\"spo2Valid\":%d,"
               "\"Ta\":%.1f,\"To\":%.1f,\"weight_kg\":%.2f}",
               (long)hr_show, hasFinger ? (int)hrValid : 0,
               (long)s2_show, hasFinger ? (int)s2Valid : 0,
               tempOK ? Ta : -99.9, tempOK ? To : -99.9,
               weight_for_json);

      client.publish(TOPIC_HEALTH, json, true);

      char buf[24];
      snprintf(buf, sizeof(buf), "%ld", (long)hr_show); client.publish(TOPIC_HR, buf, true);
      snprintf(buf, sizeof(buf), "%ld", (long)s2_show); client.publish(TOPIC_SPO2, buf, true);
      dtostrf(tempOK ? Ta : -99.9, 0, 1, buf);         client.publish(TOPIC_TA, buf, true);
      dtostrf(tempOK ? To : -99.9, 0, 1, buf);         client.publish(TOPIC_TO, buf, true);
    }
  }

  // RSSI
  if (wifiState == WifiState::CONNECTED && millis() - lastRSSI > 5000) {
    lastRSSI = millis();
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
  }

  // alive del carro
  if (client.connected() && (millis() - lastCarAlive >= CAR_ALIVE_MS)) {
    lastCarAlive = millis();
    client.publish(TOPIC_STATE, "alive");
  }
}
