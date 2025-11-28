# 🤖 Robot de Pre-Triage (ESP32 + Node-RED + WebApp)

Sistema de **pre-triaje clínico** con:

- Un **robot 2WD** con ESP32 y driver L298N controlado por **MQTT**.
- Sensores de **signos vitales** (MLX90614, MAX30102, báscula, etc.).
- Flujos en **Node-RED** para puente MQTT ⇄ WebSocket y endpoints HTTP.
- Una **WebApp** (kiosco + dashboard + vista compartida) para capturar y visualizar pacientes.

Proyecto académico de integración de **electrónica, IoT y desarrollo web**.

---

## ✨ Características

- Lectura de signos vitales en la ESP32 y envío por MQTT.
- Control remoto del carro (PWM / dirección) desde Node-RED / Web.
- Kiosco táctil para captura de datos del paciente + cuestionario.
- Cálculo de índice de **severidad** (ok / low / mid / high).
- Dashboard en tiempo real con lista de pacientes.
- Página de resultado con **código QR** para compartir.

---

## 🧱 Tecnologías

- **Hardware**: ESP32, L298N, chasis 2WD, MLX90614, MAX30102, báscula (HX711 + celda de carga).
- **Firmware**: Arduino (C/C++).
- **Backend**: Node-RED + MQTT (Mosquitto / HiveMQ / similar).
- **Frontend**: HTML + CSS + JavaScript vanilla (WebSocket, QR).

---

## 📁 Estructura del repositorio

```txt
RobotPretriaje-main/
├── Carro_PWM/
│   ├── ControlCoche_NoPWM.ino        # Carro sin control PWM
│   └── ControlCoche_PWM.ino          # Carro con PWM + MQTT
│
├── CarroySensoresComb/
│   └── CarroySensoresComNoDelay.ino  # Carro + sensores de salud en una sola ESP32
│
├── Flows Node-red/
│   ├── CARRO PWM.json                # Flow de Node-RED para control del carro
│   ├── DatosMedidos.json             # Dashboard de signos vitales
│   └── PreTriageBridge.json          # Bridge WebSocket ⇄ MQTT + endpoints HTTP
│
├── PruebasConectividadESP32/
│   ├── PruebaConectividad.ino        # Test de conexión Wi-Fi
│   └── PruebaDetectaRed.ino          # Escaneo / detección de redes
│
└── WebApp/
    ├── kiosk_ws.html                 # Kiosco de captura (WebSocket)
    ├── dashboard_ws.html             # Dashboard en tiempo real (WebSocket)
    ├── share.html                    # Vista de resultado + QR
    ├── qrcode.min.js                 # Librería para generar códigos QR
    └── LocalTests/
        ├── kiosk_local.html          # Kiosco offline (sin WS)
        ├── dashboard_local.html      # Dashboard offline
```

