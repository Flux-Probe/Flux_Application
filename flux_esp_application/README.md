# flux_esp_application

The ESP-IDF project for the Flux controller. This file covers building, wiring, configuring and using the device. For how the code is structured, see [main/README.md](main/README.md).

- [Prerequisites](#prerequisites)
- [Configuration](#configuration)
- [Build and flash](#build-and-flash)
- [Hardware and wiring](#hardware-and-wiring)
- [What happens at boot](#what-happens-at-boot)
- [Using the web dashboard](#using-the-web-dashboard)
- [HTTP API](#http-api)
- [BLE interface](#ble-interface)
- [Tuning a motor](#tuning-a-motor)
- [Logging](#logging)
- [Troubleshooting](#troubleshooting)
- [Known limitations](#known-limitations)

## Prerequisites

- ESP-IDF **v5.5** (`dependencies.lock` records v5.5.5), with `IDF_PATH` set. The project pulls in `nimble_peripheral_utils` from `${IDF_PATH}/examples/bluetooth/nimble/common/`, so the IDF examples folder must be present.
- An ESP32 board with at least 2 MB of flash.
- The hardware listed under [Hardware and wiring](#hardware-and-wiring). To test without an encoder, see [Running without sensors](#running-without-sensors).

## Configuration

### `configEnv.h` (required)

`main/mainDefs.h` includes `configEnv.h`, and `main/CMakeLists.txt` adds the repo root (`../../`) to the include path to find it. The file is gitignored because it holds credentials, so every developer creates their own at `esp_app/configEnv.h`:

```c
#ifndef _CONFIG_ENV
#define _CONFIG_ENV

#define WIFI_PASS "your-router-password"   // station-mode password (STA is currently disabled)

#endif
```

### `sdkconfig.defaults`

`sdkconfig` is generated and gitignored. `sdkconfig.defaults` is the source of truth. It sets:

| Setting | Value | Why |
| --- | --- | --- |
| `CONFIG_BT_NIMBLE_ENABLED` | `y` | NimBLE stack, BLE only (Bluedroid off) |
| `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME` | `partitions.csv` | 1.5 MB factory partition, because the image is about 1.05 MB |
| `CONFIG_FREERTOS_HZ` | `1000` | 1 ms ticks for the control loop timing |
| `CONFIG_LOG_MAXIMUM_LEVEL` | `5` | Verbose logs compiled in, enabled per module at runtime |
| `CONFIG_SPI_MASTER_ISR_IN_IRAM` | `n` | Saves about 5 KB of IRAM; the MAX31865 is polled from a task |

After you edit `sdkconfig.defaults`, delete `sdkconfig` (or run `idf.py fullclean`) so the new defaults are picked up.

`sdkconfig.defaults.mini` and `sdkconfig.defaults.dram_optimised` are size-reduction profiles copied from IDF examples. They turn off Wi-Fi or the task watchdog and **are not compatible with this app as-is**.

### Wi-Fi

The Wi-Fi settings are defined at the top of `main/webInterface/wifiSetup.c`:

| | Value |
| --- | --- |
| Mode | AP + STA (only the AP is configured) |
| AP SSID / password | `Flux-AP` / `fluxpass` (WPA/WPA2-PSK) |
| Max clients | 4 |
| Device IP | `192.168.4.1` (ESP-IDF soft-AP default) |

## Build and flash

From this directory, in an ESP-IDF terminal:

```sh
idf.py set-target esp32      # first time only
idf.py build
idf.py -p COM5 flash monitor # replace COM5 with your port; Ctrl+] exits the monitor
```

`index.html` and `style.css` are embedded in the firmware image with `EMBED_FILES`, so changes to the dashboard need a rebuild and reflash.

## Hardware and wiring

All pin and address assignments live in `main/main.c`.

### Buses

| Bus | Pins | Devices |
| --- | --- | --- |
| I2C0 (`I2C_NUM_0`) | SDA **GPIO 15**, SCL **GPIO 17** | PCA9685 at `0x40` |
| I2C1 (`I2C_NUM_1`) | SDA **GPIO 32**, SCL **GPIO 33** | ADS1115 at `0x48` (ADDR → GND) |
| SPI2 (`SPI2_HOST`) | SCLK **18**, MISO **35**, MOSI **23**, CS **26** | MAX31865 (initialisation is commented out in `app_main`) |

Both I2C buses run at 100 kHz with the ESP32's internal pull-ups enabled. **Fit external 2.2–4.7 kΩ pull-ups** on SDA and SCL. The internal pull-ups (about 45 kΩ) are marginal, and the resulting bus timeouts are what the bus-reset recovery code works around.

### Motors

Each motor uses two PCA9685 channels that feed the two inputs of an H-bridge. A positive drive command puts PWM on the "+" channel, and a negative command puts it on the "−" channel. PWM frequency is 1526 Hz, the PCA9685 maximum.

| Motor | PCA9685 channels (+ / −) | Feedback |
| --- | --- | --- |
| Motor 1 (`mtr0` in the UI) | 0 / 1 | AS5600 analog OUT → ADS1115 differential **A0–A1**, ±4.096 V range, 250 SPS |
| Motor 2 (`mtr1` in the UI) | 2 / 3 | `dummyFb`, a simulated sensor that counts 0–19 once per second |

In ADC mode the angle is calculated as `volts × 360 / 3.3`, which assumes the AS5600 analog output spans 0–3.3 V.

### Running without sensors

To bring up a motor with no encoder, edit `initMotor_FbDrivers()` in `main.c` so it uses `dummyFbInit()` for that motor, the same way motor 2 does. Then use **OPEN** mode only.

## What happens at boot

1. A 1 s delay, then NVS flash init (erased and retried if the version changed).
2. Both I2C buses are created.
3. BLE starts advertising as **`FLUX_BLE_HOST`**.
4. The PCA9685 is initialised, the ADS1115 is probed, and the motor and feedback interfaces are created. All PWM channels are forced to 0.
5. The `motor_ctrl` task starts. Every motor starts **disabled** and in **OFF** mode.
6. The Wi-Fi AP and HTTP server start, and the motor fields are registered with the dashboard.

If any step fails, `app_main` logs the error and returns. The later steps don't run, but anything already started (such as BLE) keeps running.

## Using the web dashboard

1. Join `Flux-AP` and open <http://192.168.4.1/>.
2. The page polls `/data` every 200 ms and builds a **Monitor** card and a **Controls** card for each motor.
3. To move a motor, **enable it first, then pick the mode**. While a motor is disabled, the control loop forces its mode back to OFF on every cycle, so a mode chosen before enabling is lost.
   - **Open loop:** toggle **Enable** to ON, set **Mode** to *Open*, type a value from -1.0 to 1.0 in **PWM Drive**, then click **Set PWM Target**.
   - **Position:** type a target angle in degrees in **Pos Target** and click **Set Position Target**. Then toggle **Enable** to ON and set **Mode** to *Pos*.
4. To stop, toggle **Enable** to OFF or set **Mode** to *Off*. Either one forces the drive to 0 on the next control cycle. Choosing *Off* also resets the PWM target to 0.
5. The **Raw Command** card sends any `key`/`val` pair straight to `/set`.

`state` on the Monitor card follows `mtrDriveState_e`: `2` = OPERATIONAL and `3` = FAILED. A motor latches FAILED after **5 consecutive** feedback-read or drive-write errors. Re-enabling it clears the fault.

## HTTP API

Every endpoint is a `GET`. There is **no authentication**: anyone on the AP can command the motors.

| Endpoint | Returns |
| --- | --- |
| `/` | The dashboard HTML |
| `/style.css` | Dashboard stylesheet |
| `/ping` | `pong` |
| `/data` | A JSON object of every registered monitor, e.g. `{"mtr0.pos":123.45,"mtr0.enabled":false,...}` |
| `/set?key=<key>&val=<val>` | `{"ok":true}`, or `{"ok":false,"error":"missing key or val"}` / `"unknown key"` |

Keep the whole query string under 64 characters until the parser buffer is fixed.

### Monitor keys (per motor *i*)

| Key | Meaning |
| --- | --- |
| `mtr<i>.pos` | Last feedback reading (degrees) |
| `mtr<i>.enabled` | `true`/`false` |
| `mtr<i>.mode` | `0` OFF, `1` POS, `2` OPEN |
| `mtr<i>.state` | `0` STARTUP, `1` BIT, `2` OPERATIONAL, `3` FAILED, `4` MAINTENANCE |
| `mtr<i>.posSetpoint` / `.driveSetpoint` | Current position target / open-loop target |
| `mtr<i>.dutyCmd` | Drive actually sent this cycle (-1.0 to 1.0) |
| `mtr<i>.kp` `.ki` `.kd` `.minOut` `.maxOut` `.db` | PID gains, output clamp, deadband |
| `mtr<i>.error` `.integral` `.prevError` | PID internals |

### Control keys (per motor *i*)

| Key | `val` | Notes |
| --- | --- | --- |
| `mtr<i>.enable` | `0` / `1` | Enabling also clears a latched FAILED state |
| `mtr<i>.mode` | `0` / `1` / `2` | Changing mode resets the PID integral. Forced back to `0` while the motor is disabled |
| `mtr<i>.pwmSetpoint` | `-1.0`…`1.0` | Used in OPEN mode; clamped by the driver |
| `mtr<i>.posSetpoint` | degrees | Clamped to the motor's limits (currently 0–1000) |
| `mtr<i>.gains` | `kp,ki,kd,minOut,maxOut,db` | Applied atomically between control cycles. **Send all six values**, because missing ones become 0 |
| `mtr<i>.verbLogs` | `0` / `1` | Prints loop timing every 4th cycle. A single global flag, whichever motor key you use |

Example:

```sh
curl "http://192.168.4.1/set?key=mtr0.gains&val=0.01,0.01,0,-0.4,0.4,1"
curl "http://192.168.4.1/set?key=mtr0.posSetpoint&val=180"
curl "http://192.168.4.1/set?key=mtr0.enable&val=1"   # enable before setting the mode
curl "http://192.168.4.1/set?key=mtr0.mode&val=1"
```

## BLE interface

The device advertises as **`FLUX_BLE_HOST`**. There is no pairing or encryption.

| | UUID | Properties | Purpose |
| --- | --- | --- | --- |
| Service | `0000FFE0-0000-1000-8000-00805F9B34FB` | | Flux service |
| COMMAND | `0000FFE1-…` | Write | JSON commands |
| RECIPE_WRITE | `0000FFE2-…` | Write | Recipe upload (same handler as COMMAND) |
| NOTIFICATION | `0000FFE3-…` | Read, Notify | JSON events from the device |

The advertisement carries the Alert Notification UUID (`0x1811`), not the Flux service UUID, so scan by name.

### Framing

A write is either a complete JSON document or one chunk of a larger one:

```
C<index>/<total>|<payload slice>
```

Chunks must arrive in order starting at `C0/<total>`. The payload is dispatched when chunk `total-1` arrives. The reassembled message can be up to 4095 bytes. An out-of-order chunk discards the partial message.

### Commands (app → device)

Every message has a `"type"` field.

| `type` | Fields | Status |
| --- | --- | --- |
| `RECIPE_LOAD` | `recipeId`, `probeCount`, `valveCount`, `disconnectBehaviour` (`"safe_state"` or other), `steps[]` | Parsed and logged; not executed yet |
| `STEP_ADVANCE` | `toStepIndex` | Logged only |
| `ABORT` | `safeState` (bool) | Logged only |
| `SETTINGS_UPDATE` | — | Not implemented |

Each entry in `steps[]` can contain `stepIndex`, `espControlled`, `alertOnComplete`, `durationSeconds`, and `targetTemps[]`. Each `targetTemps[]` entry can contain `probeIndex`, `minTemp`, `maxTemp` and `unit` (`"C"` or `"F"`). The limits are 20 steps and 4 targets per step. See `main/bleModule/protocolTypes.h`.

```json
{"type":"RECIPE_LOAD","recipeId":"abc","probeCount":1,"valveCount":0,
 "disconnectBehaviour":"safe_state",
 "steps":[{"stepIndex":0,"espControlled":true,"durationSeconds":600,
           "targetTemps":[{"probeIndex":0,"minTemp":64,"maxTemp":66,"unit":"C"}]}]}
```

### Notifications (device → app)

`bleNotifyPackets.c` builds three message types: `TELEMETRY`, `STEP_EVENT` and `ALERT`. Each carries a `ts` field in milliseconds since boot. Nothing calls them yet. They will be used by the recipe executor.

## Tuning a motor

1. Start in **OPEN** mode with a small drive (±0.1–0.2) to confirm direction. A positive drive should increase `pos`. If it doesn't, swap `channelPos` and `channelNeg` for that motor in `main.c`, or swap the motor leads.
2. Switch to **POS** with `ki = 0` and `kd = 0`. Raise `kp` until the motor reaches the target briskly without oscillating.
3. Add a small `ki` to remove the steady-state error. `db` (deadband, in degrees) zeroes the output and integral when the motor is close enough. Use it to stop hunting around the target.
4. `minOut` and `maxOut` cap the drive. The defaults are ±0.4.
5. Turn on **Verbose Logs** to print `overallLoopTick | semWaitTick | gainsSemTick` in microseconds. Check that the control period is what you expect.

The defaults are in `dfltPids[]` in `main.c`. Gains changed over HTTP are not saved and reset at reboot.

## Logging

Each module has a `TAG` and a `DBG` bitmask (`DBG_INFO | DBG_WARNING | DBG_ERROR` by default; see `main/loggingDefs.h`). A `LOG_x()` call prints only when its bit is set in that module's mask **and** the ESP-IDF level for that tag allows it. To get more output from one module, add `DBG_DEBUG` or `DBG_VERBOSE` to its `dbgFlag` and raise its level with `esp_log_level_set(TAG, …)`.

## Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| `fatal error: configEnv.h: No such file or directory` | Create `esp_app/configEnv.h` (see [Configuration](#configuration)). |
| `PCA9685 not found at 0x40` / `ADS1115 not found at 0x48` | Wiring, the address pins, or missing pull-ups. Check the bus pins in the table above. |
| Repeated `I2C read failed` and then `forcing STATE_FAILED` | A noisy or weak I2C bus. Add external pull-ups and shorten the wires. Re-enable the motor to clear the fault. |
| Motor runs at full output and never settles | The target is outside 0–360°, or the direction is reversed. See [Tuning a motor](#tuning-a-motor). |
| Dashboard shows "Disconnected" | The phone or PC left `Flux-AP` (some phones drop APs with no internet). Check `/ping`. |
| Image too large for partition | Lower `CONFIG_LOG_MAXIMUM_LEVEL`, or switch to `-Os` (`CONFIG_COMPILER_OPTIMIZATION_SIZE=y`). |

## Known limitations

- There is no authentication on HTTP or BLE, and the AP password is in the source.
- Position limits (0–1000) don't match the sensor range (0–360°), and there is no multi-turn tracking or wrap-around handling.
- The PID uses a fixed 8 ms `dt`, but the real loop period is longer because the ADS1115 read blocks.
- The `kd` gain is accepted but not used.
- The MAX31865 temperature driver is not wired in.
- The recipe executor is not implemented.
- There is no OTA partition.
