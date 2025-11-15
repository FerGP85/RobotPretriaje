#include <WiFi.h>
#include <esp_wifi.h>

const char* WIFI_SSID = "Fercio";
const char* WIFI_PASS = "TW1A2P3D";

const int LED = 2;

wifi_country_t MX = {
  .cc = "MX",
  .schan = 1,   // canal 1
  .nchan = 11,  // hasta 11 en 2.4 GHz
  .policy = WIFI_COUNTRY_POLICY_MANUAL
};

void printDiscReason(uint8_t r) {
  Serial.print("reason=");
  Serial.print(r);
  Serial.print(" -> ");
  switch (r) {
    case WIFI_REASON_NO_AP_FOUND: Serial.println("NO_AP_FOUND"); break;
    case WIFI_REASON_AUTH_FAIL: Serial.println("AUTH_FAIL (clave/seguridad)"); break;
    case WIFI_REASON_ASSOC_FAIL: Serial.println("ASSOC_FAIL"); break;
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: Serial.println("4WAY_TIMEOUT (WPA/PMF)"); break;
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY: Serial.println("INACTIVITY"); break;
    case WIFI_REASON_HANDSHAKE_TIMEOUT: Serial.println("HANDSHAKE_TIMEOUT"); break;
    case WIFI_REASON_ROAMING: Serial.println("ROAMING"); break;
    case WIFI_REASON_IE_IN_4WAY_DIFFERS: Serial.println("PMF/802.11w mismatch"); break;
    default: Serial.println("otro"); break;
  }
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("[WiFi] CONNECTED to %s (ch %d, BSSID %02X:%02X:%02X:%02X:%02X:%02X)\n",
                    WiFi.SSID().c_str(), WiFi.channel(),
                    info.wifi_sta_connected.bssid[0], info.wifi_sta_connected.bssid[1],
                    info.wifi_sta_connected.bssid[2], info.wifi_sta_connected.bssid[3],
                    info.wifi_sta_connected.bssid[4], info.wifi_sta_connected.bssid[5]);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("[WiFi] GOT IP: "); Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.print("[WiFi] DISCONNECTED: ");
      printDiscReason(info.wifi_sta_disconnected.reason);
      break;
    default: break;
  }
}

bool connectWithBSSIDLock() {
  Serial.println("\n=== Escaneando SSID 'Fercio' (solo 2.4 GHz) ===");
  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n <= 0) { Serial.println("No se hallaron redes."); return false; }

  int best = -1;
  int bestRSSI = -999;
  uint8_t bestBSSID[6] = {0};
  int bestChannel = 0;
  for (int i = 0; i < n; ++i) {
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    int32_t ch   = WiFi.channel(i);
    wifi_auth_mode_t auth = (wifi_auth_mode_t)WiFi.encryptionType(i);

    // Filtra SSID exacto y canal 1-11 (2.4 GHz)
    if (ssid == WIFI_SSID && ch >= 1 && ch <= 11) {
      if (rssi > bestRSSI) {
        bestRSSI = rssi; best = i; bestChannel = ch;
        WiFi.BSSID(i, bestBSSID);
      }
    }
  }

  if (best < 0) {
    Serial.println("SSID 'Fercio' no encontrado en 2.4 GHz.");
    return false;
  }

  char bssidStr[18];
  sprintf(bssidStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          bestBSSID[0],bestBSSID[1],bestBSSID[2],bestBSSID[3],bestBSSID[4],bestBSSID[5]);

  Serial.printf("Fijando BSSID %s en canal %d (RSSI %d dBm)\n", bssidStr, bestChannel, bestRSSI);

  WiFi.disconnect(true);
  delay(100);
  // begin con canal y BSSID bloqueados
  WiFi.begin(WIFI_SSID, WIFI_PASS, bestChannel, bestBSSID, true);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

bool connectStandard() {
  Serial.println("Conexión estándar (sin BSSID lock)...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  Serial.begin(115200);
  delay(200);

  WiFi.onEvent(onWiFiEvent);
  esp_wifi_set_country(&MX);              // fuerza canales MX
  WiFi.persistent(false);
  WiFi.setSleep(false);                   // sin ahorro de energía
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-diagnostico");

  // 1) Intento con bloqueo por BSSID (mitiga band steering/roaming)
  if (connectWithBSSIDLock()) {
    Serial.println("✅ Conectado con BSSID lock.");
  } else if (connectStandard()) {
    // 2) Fallback normal
    Serial.println("✅ Conectado en modo estándar.");
  } else {
    Serial.println("❌ No se pudo conectar.");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=== INFO ===");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("IP: ");   Serial.println(WiFi.localIP());
    Serial.print("GW: ");   Serial.println(WiFi.gatewayIP());
    Serial.print("Mask: "); Serial.println(WiFi.subnetMask());
    Serial.print("DNS1: "); Serial.println(WiFi.dnsIP(0));
    Serial.print("Canal: ");Serial.println(WiFi.channel());
    Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  }
}

void loop() {
  // latido si estamos conectados
  static uint32_t t0=0;
  if (WiFi.status()==WL_CONNECTED && millis()-t0>5000) {
    t0 = millis();
    digitalWrite(LED, LOW); delay(60); digitalWrite(LED, HIGH);
  }
}
