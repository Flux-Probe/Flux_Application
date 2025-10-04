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
#include "wifiSetup.h"
#include "webApp.h"
#include "mainDefs.h"
#include "nvs_flash.h"

#define TAG "MAIN"

static int logLvl = ESP_LOG_DEBUG;

typedef struct {
    mtrState_t valveMotor;
    webapp_t webApp;
    wifiConn_t wifi;
} espIf_t;

resp_t initFlash(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        LOG_E("Error initializing Flash. Will retry");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_ERROR_CHECK(ret);
        return RESP_ERR;
    }
    return RESP_OK;
}

void app_main(void)
{
    espIf_t espIf;

    resp_t sts = RESP_OK;
    esp_log_level_set(TAG, logLvl); // Setting debug
    sts = initFlash();
    RETURN_IF_ERR_LOG(sts, "Error with Nvs Flash");
    // start_ble_service();

    LOG_I("Starting MotorCtrl");
    int resp = motorCtrlMain(&espIf.valveMotor);
    LOG_D("Done MotorCtrl init %d", resp);

    /*
     * TODO: Add config parameters for WIFI to connect to
     * Call the following only Once
    */
    sts = wifi_conn_init(&espIf.wifi);
    RETURN_IF_ERR_LOG(sts, "Error with Wifi init");

    start_http_server(&espIf.webApp, &espIf.valveMotor);


    while(1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }


}
