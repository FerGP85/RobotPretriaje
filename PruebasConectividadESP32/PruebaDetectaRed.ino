#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);  // GPIO2 suele estar conectado al LED interno

  Serial.println("\n=== TEST DE FUNCIONAMIENTO ESP32 ===");
  Serial.println("1️⃣ Probando LED integrado...");
}

void loop() {
  // 1️⃣ Parpadeo del LED
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);

  // Solo ejecutar el escaneo una vez al inicio
  static bool wifiTestDone = false;
  if (!wifiTestDone) {
    Serial.println("\n2️⃣ Probando Wi-Fi (escaneo de redes)...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    if (n == 0) {
      Serial.println("❌ No se detectaron redes Wi-Fi.");
    } else {
      Serial.printf("✅ %d redes encontradas:\n", n);
      for (int i = 0; i < n; ++i) {
        Serial.printf("%d: %s (%d dBm)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        delay(10);
      }
    }
    wifiTestDone = true;
    Serial.println("\n3️⃣ Test completado. Si el LED parpadea y ves redes arriba, tu ESP32 está bien ✅");
  }
}
