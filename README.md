# Flux ESP Application

Firmware for the Flux controller: an ESP32 that runs closed-loop position control on up to two DC motors. You can monitor and command it from a browser dashboard over Wi-Fi or from the Flux mobile app over BLE.

| | |
| --- | --- |
| Target | ESP32 (2 MB flash) |
| Framework | ESP-IDF v5.5 (built and tested with v5.5.5) |
| Language | C |
| Interfaces | Wi-Fi soft-AP + HTTP dashboard, BLE GATT server (NimBLE) |
| Actuators | PCA9685 16-channel PWM driver feeding an H-bridge per motor |
| Sensors | AS5600 magnetic encoder (read through an ADS1115 ADC or directly over I2C), MAX31865 RTD converter (driver present, not yet wired into `main.c`) |

## Repository layout

```
esp_app/
├── README.md                    ← you are here
├── configEnv.h                  ← local secrets, NOT committed (see below)
└── flux_esp_application/        ← the ESP-IDF project
    ├── README.md                ← build, flash, configure, and use the device
    ├── CMakeLists.txt
    ├── partitions.csv           ← custom table: 1.5 MB factory app
    ├── sdkconfig.defaults       ← the config the project is built with
    ├── sdkconfig.defaults.*     ← experimental size-reduced profiles (not used)
    └── main/
        ├── README.md            ← firmware architecture
        ├── main.c               ← board wiring + startup sequence
        ├── motorCtrl/           ← control task, PID, fail-safe
        ├── motorDrivers/        ← motorIF_t + PCA9685 implementation
        ├── feedback/            ← feedback_t + AS5600 and dummy sensors
        ├── adc/                 ← adcIF_t + ADS1115 implementation
        ├── driver/              ← MAX31865 RTD driver
        ├── bleModule/           ← NimBLE GATT server + JSON protocol
        └── webInterface/        ← Wi-Fi AP, HTTP server, dashboard
```

## Quick start

1. Install ESP-IDF v5.5 and open an ESP-IDF terminal.
2. Create `configEnv.h` in this folder (`esp_app/`). The build fails without it:

   ```c
   #ifndef _CONFIG_ENV
   #define _CONFIG_ENV

   #define WIFI_PASS "your-router-password"   // used for station mode (currently disabled)

   #endif
   ```

3. Build, flash and open the serial monitor:

   ```sh
   cd flux_esp_application
   idf.py set-target esp32
   idf.py build
   idf.py -p COM5 flash monitor      # use your board's port
   ```

4. Join the Wi-Fi network **`Flux-AP`** (password `fluxpass`) and open <http://192.168.4.1/>.

Wiring, the HTTP and BLE protocols, and motor tuning are covered in [flux_esp_application/README.md](flux_esp_application/README.md). Module structure, the task model and how to add drivers are covered in [flux_esp_application/main/README.md](flux_esp_application/main/README.md).

## Status

This is active development firmware. Motor control, the web dashboard and BLE recipe parsing work. Recipe execution, temperature probes, and authentication on the Wi-Fi and BLE interfaces are not implemented yet. Don't leave motors powered unattended.
