# ESP App — Fix Recommendations

As of 2026-10-26 · Source: [Claude Docs version](https://claude.ai/artifact/35mmTUocwx5BZdctTwHGay)

## Summary

The firmware has a good shape: motors, feedback sensors and ADCs sit behind small function-pointer interfaces, the control loop has a fault latch, and the web dashboard builds itself from registered fields. Before it drives real hardware unattended, fix four things first:

1. **A heap buffer overflow in the web server.** `/set` copies the whole query string, up to about 500 bytes, into a 64-byte buffer. Anyone on the `Flux-AP` network can trigger it.
2. **Control-loop behaviours that can leave a motor running.** An invalid mode keeps the last drive command. A position target that the sensor can never report drives the motor at full output forever.
3. **Timing that doesn't match the PID maths.** The loop assumes 8 ms per cycle. The ADS1115 read alone blocks for 10 ms, so the integral term is scaled wrongly.
4. **No access control.** The BLE characteristics and HTTP `/set` accept commands from anyone in range, and the AP password is hard-coded in the source.

Scope: every tracked file on branch `feature/nk/cppProject` at commit `46d6981`, plus a `-Wextra` syntax-only compile of the `main` component with the toolchain recorded in `build/compile_commands.json`. Nothing was run on hardware. File references are relative to `flux_esp_application/main/`.

## Critical issues

These can corrupt memory or leave a motor powered when it should stop. Each one is a small fix.

| ID | Issue | Where | What goes wrong | Fix |
| --- | --- | --- | --- | --- |
| C1 | Heap overflow in query parsing | `webInterface/webApp.c` `getUriParam()` | `paramBuf` is 64 bytes, but the full query length + 1 is passed to `httpd_req_get_url_query_str()`. With `CONFIG_HTTPD_MAX_URI_LEN=512`, a long `/set?...` URL writes about 450 bytes past the buffer. A long gains string from the dashboard can do the same. | Pass `MAX_PARAM_LEN`, or allocate `len + 1` on the stack per request. Reject an `ESP_ERR_HTTPD_RESULT_TRUNC` result. |
| C2 | Invalid mode keeps the motor driving | `motorCtrl/motorCtrl.c` `motorControlTask()` `default:` case; `setDriveMode()` | `/set?key=mtr0.mode&val=7` is accepted. The `default` branch only logs, so `driveCmd` keeps its last value and the motor keeps running. It also logs an error on every cycle. | Validate the mode in `setDriveMode()`. In `default`, set `driveCmd = 0` and switch to `MODE_OFF`. |
| C3 | Position limits don't match the sensor range | `main.c` sets `limits` to 0–1000; `as5600.c` reports 0–360° | A target of 500 passes the clamp but can never be reached. The PID saturates at `maxOut` (0.4) and the motor spins forever. Nothing handles wrap-around at 0/360° either. | Set limits from the feedback range (0–360) or report multi-turn position. Add a stall/timeout check that latches `STATE_FAILED`. |
| C4 | Out-of-range sensor reading is fed to the PID as a position | `feedback/as5600.c` `readAS5600Deg()` | A negative ADC voltage sets `angle = -1.0` but still returns `RESP_OK`. The loop treats -1° as a real position and drives toward the setpoint. | Return `RESP_ERR` for out-of-range readings so the existing `fbFailCount` latch takes over. |
| C5 | Uninitialised `header_end` in BLE chunk parser | `bleModule/blePacketHandler.c` `blePacketHandler_onWrite()` | `sscanf("C%d/%d\|%n")` returns 2 even when the `\|` is missing, so `%n` is never written. `text + header_end` then reads an uninitialised offset, and a BLE peer can send that packet. | Initialise `header_end = -1` and require `header_end > 0`. Also reject `chunk_total <= 0` and too-large totals. |
| C6 | Watchdog fail-safe described but not configured | `sdkconfig.defaults` comment; `motorCtrl.c` | The comment promises a panic-and-reboot on a wedged I2C bus. `CONFIG_ESP_TASK_WDT_PANIC` is not set, and `motor_ctrl` never calls `esp_task_wdt_add()`, so a blocked control task is never detected. | Add `CONFIG_ESP_TASK_WDT_PANIC=y`. Subscribe the control task and call `esp_task_wdt_reset()` once per cycle. |

## Correctness and robustness

These are real bugs with a smaller blast radius than the critical list. They are grouped by subsystem and sorted by impact within each group.

### Control loop (`motorCtrl/motorCtrl.c`)

| ID | Issue | Fix |
| --- | --- | --- |
| R1 | The PID uses a fixed `dt` of 8 ms (`FREQ_125HZ`), but the real period is longer. `ctrlTaskSem` is never given, so each cycle waits the full 8 ms, then `ads1115ReadRaw()` blocks for `2 × 5 ms`, then four PCA9685 writes run. A cycle is likely about 20 ms, so the integral term is scaled by about 0.4. | Run the loop with `vTaskDelayUntil()` and pass the measured `dt` from `esp_timer_get_time()` into `pidUpdate()`. Check the real period with the existing `verbLogs` metrics. |
| R2 | `abs(pid->error)` truncates the float to an int (confirmed by `-Wabsolute-value`). An error of 1.9° counts as 1, so the deadband behaves as if it were about 1° wider than set. | Use `fabsf()`. |
| R3 | The integral has no anti-windup. While the output sits at `maxOut`, `integral` keeps growing and causes overshoot. A NaN setpoint (`strtof("nan")`) poisons `integral` until the mode changes. | Clamp `integral` or stop integrating while saturated. Reject non-finite inputs in `setTargetPos()`, `setTargetPwm()` and `setLoopGains()`. |
| R4 | `kd` and `prevError` are exposed in the UI but never used. | Implement the derivative term on measurement, or remove `kd` from the API and UI. |
| R5 | `setMotorEnable(idx, false)` calls `motorIF->enable()`, not `disable()`. Re-enabling clears `STATE_FAILED` without checking that the fault has cleared. | Call `disable()` when disabling. Probe the feedback and driver before clearing the latch. |
| R6 | Setpoints, mode and enable are written from the HTTP task with no lock. Only gains use `gainsMutex`. `CLEAR_ERRORS()` can run while `pidUpdate()` is mid-calculation. | Send commands to the control task through a FreeRTOS queue, or take the same mutex in every setter. |
| R7 | Once latched in `STATE_FAILED`, the loop still reads feedback and writes drive every cycle. Each failure resets the I2C bus and logs a warning, and the `uint8_t` fail counters wrap at 255. | Skip I/O for failed motors, apart from a periodic recovery probe. Saturate the counters. |
| R8 | `motorCtrlInit()` checks `MAX_MOTORS` instead of `numMotors`, so a build with fewer motors fails to start. | Loop to `numMotors`. |
| R29 | While a motor is disabled, the loop sets `ctrlMode = MODE_OFF` on every cycle. A mode chosen before enabling is silently lost, so the dashboard order matters: enable first, then pick the mode. | Keep the requested mode and gate only the output on `enabled`. Or reject mode changes while the motor is disabled and return an error to the UI. (The new README documents the current order.) |

### Sensor and actuator drivers

| ID | Issue | Where | Fix |
| --- | --- | --- | --- |
| R9 | On init failure, `pca9685Destroy()` deletes the shared I2C bus that `main.c` owns. | `motorDrivers/pca9685.c` | Remove only the device. The bus owner deletes the bus. |
| R10 | The ADS1115 read sleeps twice the conversion time instead of polling the OS bit or using ALERT/RDY. `ads1115CfgAlertPin()` has no `return` and is unused. `setConfig` is left NULL. There is no bus recovery on error. | `adc/ads1115.c` | Poll the config register's OS bit, or use continuous mode and read the last result. Delete or finish the alert code. |
| R11 | The AS5600 I2C timeout is 1000 ms, against an 8 ms loop. One hung read stalls every motor for a second. | `main.c` `as5600Cfgs` | Use 5–10 ms, the same as the PCA9685 and ADS1115 timeouts. |
| R12 | The MAX31865 never turns VBIAS off after a one-shot read, which defeats the self-heating design described in its header comment. | `driver/max31865.c` `triggerOneShot()` | Write `configReg` without VBIAS after reading the RTD registers. |
| R13 | The sub-zero MAX31865 polynomial skips normalising to 100 Ω (`Rt / R0 * 100`). Below 0 °C it returns about -240 °C. | `driver/max31865.c` `max31865ReadTempC()` | Multiply `rpoly` by 100 before evaluating the polynomial. |
| R14 | GPIO 4 is assigned to the ADS1115 `alertPin` and the AS5600 `resetPin`, and `EN2_PIN` in `motorCtrl.c` uses it too. None are driven yet, so this will bite when they are. | `main.c`, `motorCtrl.c` | Keep one pin map header and assert that pins are unique. |
| R15 | I2C uses the ESP32 internal pull-ups, which are about 45 kΩ. That is weak for 100 kHz over jumper wires and a likely cause of the timeouts that commit `be940b1` worked around. | `main.c` `i2cMasterCfg` | Fit external 2.2–4.7 kΩ pull-ups. Keep the bus-reset recovery as a backstop. |
| R16 | `dummyFb.h` declares `int maxTime`, but `dummyFb.c` defines `int16_t` and doesn't include its own header. `timeMs` is actually seconds. A zero `maxTime` divides by zero. One `calloc` leaks if the other fails. | `feedback/dummyFb.c` | Include the header, match the types, guard zero, and free on partial failure. |

### BLE (`bleModule/`)

| ID | Issue | Fix |
| --- | --- | --- |
| R17 | `bleNotify_telemetry()` and `bleNotify_alert()` add `snprintf` return values into `pos` without checking for truncation. Once `pos` passes the buffer size, `sizeof(buf) - pos` wraps to a huge `size_t` and the next write overflows. Four probes plus four valves is close to 512 bytes. String fields are not JSON-escaped. | Stop when `n >= remaining`. Build the JSON with cJSON, which is already linked. |
| R18 | `LOG_I("Step: %d \| %d", recipe.stepCount \| ...stepIndex)` uses `\|` where a comma was meant, so the second `%d` reads garbage. | Use a comma. |
| R19 | `buf[len] = '\0'` writes at `rx_buf[len]`, and `len` can equal `RX_BUF_SIZE`. The 256-byte MTU makes this unreachable today, but it is still an off-by-one. | Accept at most `RX_BUF_SIZE - 1` bytes in `gattSvrWrite()`. |
| R20 | `tx_buf` and `tx_len` are written by any caller of `bleNotify_send()` and read in the NimBLE host task with no lock. | Use `ble_gatts_notify_custom()` with an mbuf, so the payload travels with the notification. |
| R21 | `assert(0)` and `assert(rc == 0)` sit on paths a peer can reach, such as an unknown access op. With assertions enabled, the device aborts. | Return an ATT error and log it. |
| R22 | Subscription state (`handle_init`, `handle_sts`) is not cleared on disconnect. | Clear it in `BLE_GAP_EVENT_DISCONNECT`. |
| R23 | The device advertises the Alert Notification UUID (0x1811), not the Flux 128-bit service UUID. Apps that filter scans by service won't find it. | Put the 128-bit UUID in the scan response. |
| R24 | A partially reassembled chunk stream never times out. A dropped final chunk leaves stale state until the next `C0/...`. | Reset when more than about 2 s passes between chunks. |
| R30 | Notifications can be up to 512 bytes (`TX_BUF_SIZE`), but a notification only carries MTU − 3 bytes: at most 253 with `ATT_PREFERRED_MTU=256`. NimBLE truncates the rest, so a full `TELEMETRY` packet arrives as broken JSON. | Use the same `C<i>/<n>\|` chunk framing on the way out, or keep each message under `ble_att_mtu(conn) - 3`. |

### Web and Wi-Fi (`webInterface/`)

| ID | Issue | Fix |
| --- | --- | --- |
| R25 | `handleGetData()` passes the `snprintf` return value to `httpd_resp_send_chunk()`. If an entry is ever truncated, it reads past `chunk`. | Clamp `n` to `sizeof(chunk) - 1`. |
| R26 | `setGains()` fills any missing field with 0. Sending `"0.02,0,0"` sets `minOut = maxOut = 0`, and the motor silently stops responding. | Require all six fields, or keep the current value for any field that is omitted. |
| R27 | `verbLogs` is global but registered once per motor, so `mtr0.verbLogs` and `mtr1.verbLogs` toggle the same flag. | Register it once as `sys.verbLogs`. |
| R28 | `ESP_ERROR_CHECK` aborts on any Wi-Fi init error. A STA netif is created but never configured. | Return `RESP_ERR` and keep BLE running. Drop the STA netif until STA mode is supported. |

### Security

| ID | Issue | Fix |
| --- | --- | --- |
| S1 | The AP SSID and password (`Flux-AP` / `fluxpass`) and a home SSID are hard-coded in `wifiSetup.c` and committed to git. | Move them to `configEnv.h` or NVS. Generate a per-device AP password, for example from the MAC address, and print it on a label. |
| S2 | `/set` changes motor state over `GET` with no authentication. Any device on the AP, or any web page opened in a browser on that network, can drive the motors. | Use `POST` with a token or session check. At minimum, check the `Origin` header. |
| S3 | The BLE characteristics accept writes with no pairing or encryption. Commands are no-ops today, but that changes once the recipe executor drives valves. | Mark the write characteristics `BLE_GATT_CHR_F_WRITE_ENC` and enable LE Secure Connections with bonding. |
| S4 | The dashboard builds rows with `innerHTML` from server keys and values. | Use `textContent` for values. |

## Architecture and code quality

The interface pattern is the right one: `motorIF_t`, `feedback_t` and `adcIF_t` each hold function pointers plus a `privCtx`. Two problems need attention. Hardware configuration is spread across `main.c` and stale defines in other files. And `motorCtrl` is a hidden singleton behind a context-passing API.

| ID | Recommendation | Why |
| --- | --- | --- |
| A1 | Move all pins, bus numbers, addresses and per-motor config into one `boardConfig.h/.c`. Delete the unused `IN1_PIN`–`EN2_PIN` block in `motorCtrl.c`. | Pins are defined in three files and already collide (R14). |
| A2 | Make `motorCtrl` an honest singleton (`static` context, no `mtrCtx` export), or pass `motorCtrlCtx_t *` into every setter. | Today `motorCtrlInit()` takes a context but every setter uses the global `mtrCtx`. `mtrCtx` and `ctrlTaskSem` are also exported non-`static` globals. |
| A3 | Route all commands (web, BLE, and the future recipe executor) through one command queue owned by the control task. | This fixes R6 and gives one place to validate inputs. |
| A4 | Make module globals `static`: `dbgFlag` in `ads1115.c`, plus `handles`, `fluxBleSvc`, `flags`, `dfltFluxBleSvc`, `flux_svr_svcs`, `loadDefaults()` and `configGatts()` in `gattSvr.c`, and the Wi-Fi defaults. | Avoids symbol clashes as modules grow. |
| A5 | Separate the two logging controls. `esp_log_level_set()` takes a level, not a bitmask: `ESP_LOG_ERROR \| ESP_LOG_DEBUG` equals `ESP_LOG_VERBOSE`. `as5600Init()` writes `ESP_LOG_DEBUG` into the `DBG_*` bitmask. `loggingDefs.h` has no include guard. | The two schemes are getting mixed up. Pick one, either ESP-IDF levels per tag or the `DBG_*` mask, and use it everywhere. |
| A6 | Remove dead code: `motorDriver_L293D.*` (its header is `#error`, and `setDrive` has the wrong signature), `testTask` and its `#if 0` block, the empty `handleSettingsUpdate()`, the unused `mbedtls/base64.h` include, `DECODE_BUF_SIZE`, `I2C_CLK_SPD`, and `readRawData`/`resetData` until something calls them. | `testTask` alone holds a 4 KB stack at priority 10 and does nothing. |
| A7 | Rename the `FREQ_25HZ`/`FREQ_125HZ` macros to periods, for example `CTRL_PERIOD_MS`. | They hold milliseconds, not frequencies, and that hides R1. |
| A8 | `motorCtrl.h` includes `as5600.h`, which it doesn't need. | Control code shouldn't depend on one specific sensor. |
| A9 | Make `start_ble_service()` return a status, and have `app_main()` report a failed init by blinking an LED or starting in a safe mode instead of returning silently. | Today a BLE init failure is ignored. A motor init failure leaves BLE up with no motors and no visible error. |
| A10 | Replace the Espressif SPDX example headers on `main.c` and the BLE files with a project licence header, and add a `LICENSE` file. | The current headers make it unclear who owns the code. |
| A11 | The branch is named `cppProject` but the code is C. If a C++ port is planned, start with the interface structs, which map cleanly onto abstract classes. | Clears up the naming confusion. |

## Build, configuration and tooling

A fresh clone does not build. `mainDefs.h` includes `configEnv.h`, which is gitignored and found only because `main/CMakeLists.txt` adds `../../` to the include path.

| ID | Issue | Fix |
| --- | --- | --- |
| B1 | `configEnv.h` is required but not in the repo, and its expected contents (`WIFI_PASS`, `DEBUG`, `USE_DUMMY_FB`) aren't documented. `DEBUG` and `USE_DUMMY_FB` are never read. | Commit a `configEnv.h.example` (the new README describes it). Better still, replace it with a `Kconfig.projbuild` menu so `idf.py menuconfig` sets the values. Remove the unused macros. |
| B2 | `dependencies.lock` stores an absolute Windows path (`C:\Espressif\frameworks\esp-idf-v5.5\...`). | Regenerate it on each machine, or gitignore it while the only dependency is a local IDF example component. |
| B3 | `REQUIRES` lists `esp_https_server` and `esp_driver_mcpwm`, but only `esp_http_server` is used, and MCPWM belongs to the dead L293D driver. | Trim the list to what the code uses. |
| B4 | `sdkconfig.defaults.mini` and `sdkconfig.defaults.dram_optimised` are copied from IDF examples. `.mini` disables Wi-Fi, and `.dram_optimised` disables the task watchdog and Wi-Fi/BLE coexistence. Both break this app if picked by mistake. | Delete them, or document them as experimental profiles. |
| B5 | The image is about 1.05 MB in a 1.5 MB `factory` partition on 2 MB flash, with no OTA slots. | Plan a two-slot OTA table on 4 MB flash before the image grows further. |
| B6 | `sdkconfig.defaults` sets `CONFIG_LOG_MAXIMUM_LEVEL=5` (verbose compiled in) and the debug `-Og` build, both of which grow the image. | Add a release profile with `-Os` and a maximum log level of INFO. |
| B7 | `-Wextra` reports four warnings in `main`: `-Wabsolute-value` in `motorCtrl.c` (R2), two unused variables in `ads1115.c`, and `struct ble_gap_event` used before it is declared in `fluxBleService.h`. | Fix them, then add `-Werror` for the `main` component with `target_compile_options(${COMPONENT_LIB} PRIVATE -Wextra -Werror)`. |
| B8 | There is no CI, formatter or static analysis. | Add a GitHub Actions build using `espressif/esp-idf-ci-action`, a `.clang-format`, and `idf.py clang-check` or cppcheck. |

## Testing and verification gaps

There are no automated tests. Most of the bugs above are in pure functions that can be tested on a PC without hardware.

- [ ] **Host unit tests** (ESP-IDF `linux` target or plain Unity + CMake): `pidUpdate()` covering deadband, saturation, windup and NaN; the BLE chunk reassembler covering a missing `|`, out-of-order chunks and overflow; `urlDecode()`; and the `setGains()` parser.
- [ ] **Driver tests against fakes:** run `motorIF_t` and `feedback_t` against a fake I2C layer to check the `STATE_FAILED` latch after 5 failures and the recovery on re-enable.
- [ ] **On-target smoke test:** a `pytest-embedded` script that boots, checks for the `HTTP server ready` log line, calls `/ping` and `/data`, and sends one `/set`.
- [ ] **Loop timing check:** turn on `verbLogs` and record `overallLoopTick` and `semWaitTick` to confirm the real control period (R1). Repeat after any driver change.
- [ ] **Fault injection on hardware:** unplug the AS5600, ADS1115 and PCA9685 one at a time while a motor is enabled. Confirm the motor stops within 5 cycles and the dashboard shows `state = 3` (`STATE_FAILED`).
- [ ] **Fuzz the parsers:** feed random strings to `/set` and to the BLE `COMMAND` characteristic. This would have found C1 and C5.

## Prioritized action plan

Do the phases in order. Phase 1 is small, mostly one-line fixes, and should land before the next powered test. The effort figures are rough estimates.

### Phase 1: Safety and memory (about 1 day)

- [ ] C1: bound the `/set` query buffer
- [ ] C2: validate the mode, and zero the drive in the `default` case
- [ ] C3, C4: set limits to match the sensor range, and treat out-of-range readings as failures
- [ ] C5: initialise and check `header_end` in the BLE chunk parser
- [ ] C6: enable the watchdog panic and subscribe `motor_ctrl`
- [ ] R2, R18: `fabsf()`, and fix the `LOG_I` comma
- [ ] R17, R25: bound the `snprintf` accumulation in the BLE and web JSON builders

### Phase 2: Control quality (about 2–3 days)

- [ ] R1: fixed-rate loop with measured `dt`, and confirm with `verbLogs`
- [ ] R3, R4: anti-windup, NaN rejection, and a decision on `kd`
- [ ] R6, A3: a command queue in front of the control task
- [ ] R10, R11: a non-blocking ADS1115 read, and short I2C timeouts
- [ ] R15: external I2C pull-ups
- [ ] R5, R7, R26: fix enable/disable and latch behaviour, and the gains parser

### Phase 3: Security (about 2 days)

- [ ] S1: credentials out of source, with a per-device AP password
- [ ] S2: `POST` plus a token for `/set`
- [ ] S3: encrypted BLE writes with bonding

### Phase 4: Maintainability (ongoing)

- [ ] B1: `configEnv.h.example` or Kconfig, so a fresh clone builds
- [ ] A1, A6: one board-config file, and remove dead code
- [ ] B7, B8: zero warnings with `-Werror`, plus CI
- [ ] Host unit tests for the PID, the parsers and the fail-safe latch
- [ ] R12, R13: MAX31865 fixes before wiring the RTD into `main.c`

Architecture and usage are documented separately in the README files in the repo: [README.md](README.md), [flux_esp_application/README.md](flux_esp_application/README.md) and [flux_esp_application/main/README.md](flux_esp_application/main/README.md).
