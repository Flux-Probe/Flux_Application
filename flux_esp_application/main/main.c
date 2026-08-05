/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c_master.h"
#include "motorCtrl.h"
#include "wifiSetup.h"
#include "fluxBleService.h"
#include "webApp.h"
#include "mtrWebBindings.h"
#include "mainDefs.h"
#include "nvs_flash.h"
#include "motorDriver_PCA9685.h"

#define TAG "MAIN"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_ERROR;

static int logLvl = ESP_LOG_DEBUG;

#define VAL_LIM 1000

/****************************************************
 *
 * I2C Macros and Configurations
 *
 ****************************************************/
#define I2C_MAX_PORTS 2
#define I2C_1_SDA_PIN 21
#define I2C_1_SCL_PIN 22
#define I2C_2_SDA_PIN 0
#define I2C_2_SCL_PIN 0

typedef struct {
    i2c_master_bus_config_t busCfg;
    i2c_master_bus_handle_t bus;
    bool enable;
}i2cBus_t;

static i2cBus_t i2cMasterCfg[] = {
    [0] = {
        .busCfg = {
            .i2c_port       = I2C_NUM_0,
            .sda_io_num     = I2C_1_SDA_PIN,
            .scl_io_num     = I2C_1_SCL_PIN,
            .clk_source     = I2C_CLK_SRC_DEFAULT,
            // .flags.enable_internal_pullup = 1,
        },
        .enable = true,
    },
    [1] = {
        .busCfg = {
            .i2c_port       = I2C_NUM_1,
            .sda_io_num     = I2C_2_SDA_PIN,
            .scl_io_num     = I2C_2_SCL_PIN,
            .clk_source     = I2C_CLK_SRC_DEFAULT,

        },
        .enable = false,
    },
};

typedef struct {
    motorCtrlCtx_t  motorCtrl;
    webApp_t        webApp;
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

resp_t initI2CPorts(void)
{
    for (int i = 0; i < I2C_MAX_PORTS; i++) {
        if (!i2cMasterCfg[i].enable) {
            continue;
        }

        esp_err_t err = i2c_new_master_bus(&i2cMasterCfg[i].busCfg, &i2cMasterCfg[i].bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
            return RESP_ERR;
        }
    }

    return RESP_OK;
}

void app_main(void)
{
    resp_t sts = RESP_OK;
    esp_log_level_set(TAG, logLvl); // Setting debug
    sts = initFlash();
    RETURN_IF_ERR_LOG(sts, "Error with Nvs Flash");

    start_ble_service(&espIF.bleSvc);

    sts = motorCtrlInit(&espIF.motorCtrl);
    RETURN_IF_ERR_LOG(sts, "Done MotorCtrl init %d", sts);

    sts = wifi_conn_init(&espIF.wifi);
    RETURN_IF_ERR_LOG(sts, "Error with Wifi init");

    sts = startHttpServer(&espIF.webApp);
    RETURN_IF_ERR_LOG(sts, "Error when starting webApp");

    motorCtrlRegisterWebBindings(&espIF.webApp, &espIF.motorCtrl);

    sts = initI2CPorts();
    RETURN_IF_ERR_LOG(sts, "Error initalizing I2C Buses");
    xTaskCreate(testTask, "testTask", 4096, &espIF, 10, NULL);
}
