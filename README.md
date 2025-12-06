# Robot Pre-Triage (ESP32 + Node-RED + WebApp)

Sistema de **pre-triaje clínico** con un robot 2WD controlado por MQTT, medición de signos vitales y una WebApp (kiosco + dashboard + página compartible).

## Qué hace
- **ESP32**: controla el carro (L298N) y publica mediciones por **MQTT**.
- Sensores:
  - **MAX30102** (HR/SpO₂)
  - **MLX90614** (temperatura)
  - **Báscula BLE** (peso)
  - **Ultrasonido** (detección de borde/caída)
- **Node-RED**: puente **MQTT ⇄ WebSocket** y endpoint **HTTP** para consultar resultados.
- **WebApp**:
  - `kiosk_ws.html` (captura + medición)
  - `dashboard_ws.html` (visualización en tiempo real)
  - `share.html` (resultado por ID + QR)

---

## Estructura del repo
```txt
CarroySensoresComb/
  CarroySensoresComNoDelay.ino     # Firmware principal (recomendado)
Flows Node-red/
  PreTriageBridge.json             # WS <-> MQTT + API
  CARRO PWM.json                   # Control del carro
  DatosMedidos.json                # Dashboard/mediciones
WebApp/
  kiosk_ws.html
  dashboard_ws.html
  share.html
  qrcode.min.js
  LocalTests/                      # Pruebas antiguas (desactualizadas)
Outdated_CarroPWM/                 # Firmwares viejos (referencia)
PruebasConectividadESP32/          # Tests WiFi/escaneo redes
DiseñoRobot/                       # STL del diseño (impresión 3D)
```

> Nota: `Outdated_CarroPWM/` y `WebApp/LocalTests/` son versiones antiguas; usar los archivos actuales.

---

## Requisitos
- **Broker MQTT** (Mosquitto/HiveMQ/otro)
- **Node-RED** (en PC o Raspberry Pi)
- Arduino IDE / PlatformIO para compilar el firmware ESP32

---

## Configuración rápida

### 1) ESP32 (firmware)
Abre:
- `CarroySensoresComb/CarroySensoresComNoDelay.ino`

Edita tus credenciales:
```cpp
const char* WIFI_SSID   = "TuSSID";
const char* WIFI_PASS   = "TuPassword";
const char* MQTT_SERVER = "IP_o_HOST_broker";
const int   MQTT_PORT   = 1883;
```

Sube el sketch y verifica conexión en el monitor serie.

### 2) Node-RED
1. Importa los flows desde `Flows Node-red/`.
2. Ajusta el nodo del **broker MQTT** (host/port/credenciales).
3. Deploy.

### 3) WebApp
En `WebApp/kiosk_ws.html` y `WebApp/dashboard_ws.html` cambia:
```js
const WS_URL = "ws://<IP_NODE_RED>:1880/ws/pretriage";
```

Abrir:
- `http://<IP_NODE_RED>:1880/kiosk_ws.html`
- `http://<IP_NODE_RED>:1880/dashboard_ws.html`
- `http://<IP_NODE_RED>:1880/share.html?id=<ID>`

---

## Topics MQTT (referencia)
- Control carro: `car/cmd` (F/B/L/R/S, SPD:0-255, etc.)
- Estado carro: `car/state`
- Salud (global): `esp32/health` (JSON)
- Subtopics: `esp32/health/hr`, `esp32/health/spo2`, `esp32/health/ta`, `esp32/health/to`
- Peso: `esp32/scale/weight_kg`

---
