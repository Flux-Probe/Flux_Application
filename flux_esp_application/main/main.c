/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// BLE MODULE
#include "motorCtrl.h"
#include "wifiSetup.h"
#include "fluxBleService.h"
#include "webApp.h"
#include "mainDefs.h"
#include "nvs_flash.h"

#define TAG "MAIN"

static int logLvl = ESP_LOG_DEBUG;

#define VAL_LIM 1000

typedef struct {
    motorCtrlCtx_t  motorCtrl;
    webapp_t        webApp;
    wifiConn_t      wifi;
    bleSvc_t        bleSvc;
} espIf_t;

static espIf_t espIF;

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

void testTask(void *arg)
{
    espIf_t *espIf = (espIf_t *) arg;
    int32_t cntr = 0;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        cntr ++;

        // Dummy task to change the values of each characteristic to debug on the app
#if 0
        if (cntr % 1 == 0) {
            espIf->bleSvc.chars[CURR_TEMP_CHR].value_f += 1;
            if (espIf->bleSvc.chars[CURR_TEMP_CHR].value_f > VAL_LIM) {
                espIf->bleSvc.chars[CURR_TEMP_CHR].value_f = 0;
            }
            notifyCharUpdate(espIF.bleSvc.chars[CURR_TEMP_CHR]);
        }
        if (cntr % 5 == 0)
        {
            espIf->bleSvc.chars[TARGET_TEMP_CHR].value_f += 0.5;
            if (espIf->bleSvc.chars[TARGET_TEMP_CHR].value_f > VAL_LIM) {
                espIf->bleSvc.chars[TARGET_TEMP_CHR].value_f = 0;
            }
            notifyCharUpdate(espIF.bleSvc.chars[TARGET_TEMP_CHR]);
        }
        if (cntr % 10 == 0)
        {
            espIf->bleSvc.chars[MTR_CURRENT_CHR].value_f += 0.5;
            if (espIf->bleSvc.chars[MTR_CURRENT_CHR].value_f > VAL_LIM) {
                espIf->bleSvc.chars[MTR_CURRENT_CHR].value_f = 0;
            }
            notifyCharUpdate(espIF.bleSvc.chars[MTR_CURRENT_CHR]);
        }
        if (cntr % 2 == 0)
        {
            espIf->bleSvc.chars[MTR_POS_CHR].value_i += 1;
            if (espIf->bleSvc.chars[MTR_POS_CHR].value_i > VAL_LIM) {
                espIf->bleSvc.chars[MTR_POS_CHR].value_i = 0;
            }
            notifyCharUpdate(espIF.bleSvc.chars[MTR_POS_CHR]);
        }
        LOG_I("Characterisitcs: %.2f|%.2f|%.2f|%d",
              espIf->bleSvc.chars[CURR_TEMP_CHR].value_f,
              espIf->bleSvc.chars[TARGET_TEMP_CHR].value_f,
              espIf->bleSvc.chars[MTR_CURRENT_CHR].value_f,
              espIf->bleSvc.chars[MTR_POS_CHR].value_i
        );
#endif
    }
}

void app_main(void)
{

    resp_t sts = RESP_OK;
    esp_log_level_set(TAG, logLvl); // Setting debug
    sts = initFlash();
    RETURN_IF_ERR_LOG(sts, "Error with Nvs Flash");

    start_ble_service(&espIF.bleSvc);

    LOG_I("Starting MotorCtrl");
    int resp = motorCtrlInit(&espIF.motorCtrl);
    LOG_D("Done MotorCtrl init %d", resp);

    /*
     * TODO: Add config parameters for WIFI to connect to
     * Call the following only Once
    */
    sts = wifi_conn_init(&espIF.wifi);
    RETURN_IF_ERR_LOG(sts, "Error with Wifi init");

    start_http_server(&espIF.webApp, &espIF.motorCtrl);

    xTaskCreate(testTask, "testTask", 4096, &espIF, 10, NULL);


}
