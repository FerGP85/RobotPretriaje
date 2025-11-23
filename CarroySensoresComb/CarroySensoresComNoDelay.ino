/****************************************
 * Includes comunes
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
 * MQTT (mismo broker para todo)
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
const char* TOPIC_SCALE   = "esp32/scale/weight_kg";   // peso por MQTT

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

/* ====== PWM ====== */
// Usamos analogWrite() como en tu código original
int duty = 160;            // 0..255 velocidad por defecto
const int KICK_DUTY = 255; // golpe inicial opcional
const int KICK_MS   = 120; // duración del kick (ms)
bool useKick = false;      // true si quieres el "golpe" inicial

// Prototipo de callback MQTT (para poder usarlo en mqttSetup)
void mqttCallback(char* topic, byte* payload, unsigned int length);

/****************************************
 * I²C único (Wire) para sensores
 ****************************************/
#define I2C_SDA    21
#define I2C_SCL    22
#define I2C_FREQ   100000   // 100 kHz

/****************************************
 * Sensores
 ****************************************/
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // 0x5A
bool mlxOK = false;

MAX30105 particleSensor;
bool maxOK = false;

// Del algoritmo MAX
#define SAMPLE_DELAY_MS (1000 / FreqS)   // ~40 ms (25 Hz)

/****************************************
 * Calibración MLX
 ****************************************/
const float MLX_OFFSET_C = 2.5f;   // offset para To (frente ~2 cm)

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

bool fingerPresent();
inline void resetFilters() { hr_f = -1.0f; spo2_f = -1.0f; }
// Estado global de dedo y HR/SpO2 (último cálculo terminado)
bool hasFingerGlobal     = false;
volatile bool hrSpo2Ready = false;
int32_t hr_last          = -1;
int32_t spo2_last        = -1;
int8_t hrValid_last      = 0;
int8_t spo2Valid_last    = 0;


/****************************************
 * Tiempo / timers
 ****************************************/
uint32_t lastUpdateMs   = 0;
const uint32_t UPDATE_MS = 2000;   // intervalo de envío de salud

uint32_t lastRSSI       = 0;

float     lastWeightKg  = NAN;
uint32_t  lastWeightAt  = 0;
const uint32_t WEIGHT_FRESH_MS = 8000;

// "alive" del carro
uint32_t lastCarAlive   = 0;
const uint32_t CAR_ALIVE_MS = 5000;
uint32_t lastCarCmdMs = 0;

/****************************************
 * Utilidades sensores
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

bool initMAX30102() {
  // Inicializa el sensor en el mismo bus I2C
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    return false;
  }

  // Sube brillo y baja tasa de muestreo a 25 Hz (igual que FreqS del algoritmo)
  byte ledBrightness = 0x8F;  // antes 0x1F
  byte sampleAverage = 4;
  byte ledMode       = 2;     // Rojo + IR
  int  sampleRate    = 25;    // Hz, para que coincida con FreqS=25
  int  pulseWidth    = 411;
  int  adcRange      = 16384;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  // Usa el mismo brillo en los dos LEDs
  particleSensor.setPulseAmplitudeRed(ledBrightness);
  particleSensor.setPulseAmplitudeIR(ledBrightness);
  particleSensor.setPulseAmplitudeGreen(0);  // apagado

  return true;
}

void hrSpO2Service() {
  if (!maxOK) return;

  static uint32_t irBuffer[BUFFER_SIZE];
  static uint32_t redBuffer[BUFFER_SIZE];
  static int idx = 0;

  // Si no hay dedo, reseteamos índice y salimos rápido
  if (!hasFingerGlobal) {
    idx = 0;
    return;
  }

  // Actualizar FIFO del sensor
  particleSensor.check();

  // Leer todas las muestras disponibles SIN bloqueos largos
  while (particleSensor.available()) {
    redBuffer[idx] = particleSensor.getRed();
    irBuffer[idx]  = particleSensor.getIR();
    particleSensor.nextSample();
    idx++;

    if (idx >= BUFFER_SIZE) {
      int32_t hr; int8_t hrValid;
      int32_t spo2; int8_t spo2Valid;

      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, BUFFER_SIZE, redBuffer,
        &spo2, &spo2Valid, &hr, &hrValid);

      hr_last        = hr;
      hrValid_last   = hrValid;
      spo2_last      = spo2;
      spo2Valid_last = spo2Valid;
      hrSpo2Ready    = true;

      idx = 0;  // empezamos de nuevo para el siguiente cálculo
      break;    // solo calculamos una vez por buffer lleno
    }
  }
}


bool readTemps(double &Ta, double &To) {
  if (!mlxOK) { Ta = NAN; To = NAN; return false; }
  Ta = mlx.readAmbientTempC();
  To = mlx.readObjectTempC();
  To += MLX_OFFSET_C;
  return isfinite(Ta) && isfinite(To);
}

/****************************************
 * Detección de dedo
 ****************************************/
bool fingerPresent() {
  if (!maxOK) return false;

  const int N = 16;
  uint32_t ir;
  double sum = 0.0, sum2 = 0.0;
  int samples = 0;

  // Leemos hasta N muestras SI están disponibles, sin esperar
  for (int i = 0; i < N; i++) {
    particleSensor.check();
    if (!particleSensor.available()) continue;

    ir = particleSensor.getIR();
    particleSensor.nextSample();

    sum  += ir;
    sum2 += (double)ir * (double)ir;
    samples++;
  }

  // Si casi no hay muestras, asumimos que no hay dedo
  if (samples < 4) {
    hasFingerGlobal = false;
    return false;
  }

  double mean = sum / samples;
  double var  = (sum2 / samples) - (mean * mean);
  double stdv = (var > 0) ? sqrt(var) : 0;

  const double MEAN_MIN = 5000.0;
  const double STD_MIN  = 5.0;

  bool present = (mean >= MEAN_MIN) && (stdv >= STD_MIN);

  hasFingerGlobal = present;

  // Debug (si ves que estorba, luego lo comentas)
  Serial.print("[finger] mean=");
  Serial.print(mean);
  Serial.print(" std=");
  Serial.print(stdv);
  Serial.print(" samples=");
  Serial.print(samples);
  Serial.print(" -> ");
  Serial.println(present ? "DED0" : "NO");

  return present;
}


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
    case 17: return "IE_INVALID";
    case 23: return "MIC_FAILURE";
    case 24: return "4WAY_HANDSHAKE_TIMEOUT2";
    case 29: return "ASSOC_FAIL";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL2";
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
    Serial.println("[WiFi] No se encontró el SSID objetivo en el escaneo.");
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
    Serial.printf("[WiFi] Conectando a \"%s\" anclado a canal %d / BSSID ...\n", WIFI_SSID, targetChannel);
    WiFi.begin(WIFI_SSID, WIFI_PASS, targetChannel, targetBSSID, true);
  } else {
    Serial.printf("[WiFi] Conectando a \"%s\" (sin anclaje)...\n", WIFI_SSID);
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
      Serial.println("[WiFi] Timeout conectando. Re-escanear y reintentar...");
      wifiFindAP(WIFI_SSID);
      if (useBSSIDLock == true) useBSSIDLock = false;
      wifiState = WifiState::IDLE;
      startedAt = 0;
      backoffMs = min<uint16_t>(backoffMs * 2, 30000);
      nextAction = millis() + backoffMs;
    } else {
      nextAction = millis() + 300;
    }
    return;
  }

  if (useBSSIDLock && (targetChannel == 0 || targetBSSID[0] == 0)) {
    wifiFindAP(WIFI_SSID);
  }
  wifiStartConnect();
  nextAction = millis() + backoffMs;
  backoffMs = min<uint16_t>(backoffMs * 2, 30000);
}

/****************************************
 * MQTT helpers con backoff
 ****************************************/
void mqttSetup(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);
  client.setBufferSize(256);
  client.setCallback(mqttCallback);      // <-- para comandos del carro
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
    // Suscribir comandos del carro
    client.subscribe(TOPIC_CMD);
    // Estado inicial
    client.publish(TOPIC_STATE, "online");
  } else {
    Serial.printf("falló rc=%d\n", client.state());
    backoff = backoff < 30000 ? backoff * 2 : 30000;
  }
  nextTry = millis() + backoff;
}

/****************************************
 * ===== BLE Mi Body Scale =====
 ****************************************/
static const char* UUID_181B = "0000181b-0000-1000-8000-00805f9b34fb"; // Body Composition
static const char* UUID_181D = "0000181d-0000-1000-8000-00805f9b34fb"; // Weight Scale

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
  Serial.printf("%.2f\n", kg);
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
      String uuid = dev.getServiceDataUUID(i).toString();
      if (!uuid.equalsIgnoreCase(UUID_181B) && !uuid.equalsIgnoreCase(UUID_181D)) continue;

      String data = dev.getServiceData(i);
      const uint8_t* sd = (const uint8_t*)data.c_str();
      int sl = data.length();

      float kg = pickWeightFrom181x(sd, sl);
      if (!isnan(kg)) {
        publishWeight(kg);
      }
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

  const uint32_t SCAN_PERIOD_MS      = 5000; // cada 5 s
  const uint32_t NO_SCAN_AFTER_CMD_MS = 2000; // 2 s después de mover el carro, NO escanees

  // Si hace poco se envió un comando al carro, no bloquees con BLE
  if (now - lastCarCmdMs < NO_SCAN_AFTER_CMD_MS) {
    return;
  }

  if (now - lastScan < SCAN_PERIOD_MS) {
    return; // nada que hacer todavía
  }
  lastScan = now;

  // Esta parte bloquea ~1 s, pero sólo cuando no estamos controlando el carro
  g_bleScan->start(1 /*sec*/, false);
  g_bleScan->clearResults();
}

/****************************************
 * ==== Código del carro (L298N) ====
 ****************************************/
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

void setPWM_A(int d){ if (d<0) d=0; if (d>255) d=255; analogWrite(ENA, d); }
void setPWM_B(int d){ if (d<0) d=0; if (d>255) d=255; analogWrite(ENB, d); }

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
  setDirA(-1); setDirB(-1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}
void driveBackward() {
  setDirA(+1); setDirB(+1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}
void turnLeft()  {
  setDirA(+1); setDirB(-1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}
void turnRight() {
  setDirA(-1); setDirB(+1);
  applyKickIfNeeded(duty);
  setPWM_A(duty); setPWM_B(duty);
}

void publishCarState(const char* state) {
  if (client.connected()) {
    client.publish(TOPIC_STATE, state);
  }
}

/****************************************
 * MQTT callback: comandos del carro
 ****************************************/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Solo nos interesan los mensajes de car/cmd
  if (strcmp(topic, TOPIC_CMD) != 0) return;

  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String up = msg;
  up.toUpperCase();

  if (up == "F") {
    driveForward();  publishCarState("forward");
    lastCarCmdMs = millis();    // <<< NUEVO
    return;
  }
  if (up == "B") {
    driveBackward(); publishCarState("backward");
    lastCarCmdMs = millis();
    return;
  }
  if (up == "L") {
    turnLeft();      publishCarState("left");
    lastCarCmdMs = millis();
    return;
  }
  if (up == "R") {
    turnRight();     publishCarState("right");
    lastCarCmdMs = millis();
    return;
  }
  if (up == "S" || up=="STOP") {
    stopMotors();  publishCarState("stop");
    lastCarCmdMs = millis();
    return;
  }
  if (up == "COAST") {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    setPWM_A(0); setPWM_B(0);
    publishCarState("coast");
    return;
  }
  if (up == "BRAKE") { stopMotors(); publishCarState("brake"); return; }

  // Ajuste de velocidad: SPD:0..255
  if (up.startsWith("SPD:")) {
    int val = msg.substring(4).toInt();
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    duty = val;
    String s = String("speed=") + String(duty);
    publishCarState(s.c_str());
    lastCarCmdMs = millis();
    return;
  }
}

/****************************************
 * Setup
 ****************************************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[START] ESP32 Car + Health + BLE Scale");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  scanI2C(Wire, "Wire (SDA=21, SCL=22)");

  mlxOK = initMLX();
  Serial.println(mlxOK ? "MLX90614 OK (0x5A)" : "MLX90614 NO detectado");

  maxOK = initMAX30102();
  Serial.println(maxOK ? "MAX30102 OK (0x57 tipico)" : "MAX30102 NO detectado");

  // Motores: pines como salida y estado seguro
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  stopMotors();

  // WiFi + MQTT
  wifiInitStation();
  wifiFindAP(WIFI_SSID);
  wifiStartConnect();

  mqttSetup();

  // BLE báscula
  scaleInit();
}

/****************************************
 * Loop
 ****************************************/
void loop() {
  // 1) Servicio rápido de HR/SpO2
  hrSpO2Service();

  // 2) Servicios de red
  wifiService();
  mqttService();

  // 3) BLE báscula
  scaleService();

  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_MS) {
    lastUpdateMs = now;

    // Temperaturas
    double Ta = NAN, To = NAN;
    bool tempOK = readTemps(Ta, To);

    // Actualizar detección de dedo (también llena hasFingerGlobal)
    bool hasFinger = fingerPresent();

    int32_t hr = -1; int8_t hrValid = 0;
    int32_t s2 = -1; int8_t s2Valid = 0;

    if (hasFinger && hrSpo2Ready) {
      // Consumimos la última medición disponible
      hr        = hr_last;
      hrValid   = hrValid_last;
      s2        = spo2_last;
      s2Valid   = spo2Valid_last;
      hrSpo2Ready = false;

      Serial.print("[hr/spo2] hr=");
      Serial.print(hr);
      Serial.print(" v=");
      Serial.print((int)hrValid);
      Serial.print(" spo2=");
      Serial.print(s2);
      Serial.print(" v=");
      Serial.println((int)s2Valid);

      updateFilters(hr, hrValid, s2, s2Valid);
    } else {
      resetFilters();
    }

    int32_t hr_show  = hasFinger ? ((hr_f > 0) ? (int32_t)lroundf(hr_f) : hr) : -1;
    int32_t s2_show  = hasFinger ? ((spo2_f > 0) ? (int32_t)lroundf(spo2_f) : s2) : -1;

    float weight_for_json = (!isnan(lastWeightKg) && (now - lastWeightAt <= WEIGHT_FRESH_MS))
                              ? lastWeightKg : -1.0f;

    // Debug Serial
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

    // MQTT salud
    if (client.connected()) {
      char json[240];
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
      // peso ya va por TOPIC_SCALE cuando llega el anuncio BLE
    }
  }

  // Telemetría de señal WiFi
  if (wifiState == WifiState::CONNECTED && millis() - lastRSSI > 5000) {
    lastRSSI = millis();
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
  }

  // "Latido" del carro
  if (client.connected() && (millis() - lastCarAlive >= CAR_ALIVE_MS)) {
    lastCarAlive = millis();
    client.publish(TOPIC_STATE, "alive");
  }
}
