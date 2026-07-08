/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "motorCtrl.h"
#include "wifiSetup.h"
#include "fluxBleService.h"
#include "webApp.h"
#include "mtrWebBindings.h"
#include "mainDefs.h"
#include "nvs_flash.h"
#include "tempSensor.h"
#include "ads1115.h"
#include "i2cTypes.h"

#define TAG "MAIN"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_ERROR | DBG_WARNING;

static int logLvl = ESP_LOG_VERBOSE;

#define VAL_LIM 1000

typedef struct {
    motorCtrlCtx_t  motorCtrl;
    webApp_t        webApp;
    wifiConn_t      wifi;
    bleSvc_t        bleSvc;
    tempSensor_t    *temp;
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

    float temp_1 = 0;
    float temp_2 = 0;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        cntr ++;

        espIF.temp->readTemp(espIF.temp, 0, &temp_1);
        espIF.temp->readTemp(espIF.temp, 1, &temp_2);

        LOG_I("Temp 1: %.2f | Temp 2: %.2f", temp_1, temp_2);

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

    LOG_I("info");
    LOG_W("warn");
    LOG_E("err");
    LOG_D("dbg");
    // start_ble_service(&espIF.bleSvc);

    // sts = motorCtrlInit(&espIF.motorCtrl);
    // RETURN_IF_ERR_LOG(sts, "Done MotorCtrl init %d", sts);

    // sts = wifi_conn_init(&espIF.wifi);
    // RETURN_IF_ERR_LOG(sts, "Error with Wifi init");

    // sts = startHttpServer(&espIF.webApp);
    // RETURN_IF_ERR_LOG(sts, "Error when starting webApp");

    // motorCtrlRegisterWebBindings(&espIF.webApp, &espIF.motorCtrl);
    ads1115Cfg_t tempCfg = {
        .dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR,
        .i2cCfg = {
            .masterCfg.i2c_port    = I2C_NUM_1,
            .masterCfg.sda_io_num  = 21,
            .masterCfg.scl_io_num  = 22,
            .masterCfg.clk_source  = I2C_CLK_SRC_DEFAULT,
            .devCfg.dev_addr_length    = I2C_ADDR_BIT_LEN_7,
            .devCfg.device_address     = ADS1115_ADDR_0,
            .devCfg.scl_speed_hz       = 400000,
        },
        .dr = ADS1115_128SPS,
        .vRef = 3.3,
        .rRef = 4700,
        .pga = ADS1115_PGA_2048,
        .numChannels = 2,
        .alertPin = 4,
    };

    espIF.temp = ads1115Init(tempCfg);
    if (espIF.temp == NULL) {
        LOG_E("ADS1115 init failed — temperature readings unavailable");
    }

#ifdef DEBUG
    if (espIF.temp != NULL) {
        xTaskCreate(testTask, "testTask", 4096, &espIF, 10, NULL);
    }
#endif

}
