/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// BLE MODULE
// #include "fluxBleService.h"
#include "motorCtrl.h"
#include "mainDefs.h"

#define TAG "MAIN"
#define LOG_LVL logLvl

static int logLvl = ESP_LOG_DEBUG;

void app_main(void)
{
    esp_log_level_set(TAG, logLvl); // Setting debug
    // start_ble_service();

    LOG_I("Starting MotorCtrl");
    int resp = motorCtrlMain();

    LOG_D("Done MotorCtrl init %d", resp);

    while(1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // LOG_I("Current temp %.2f| target_temp %.2f| mtr pos");
    }


}
