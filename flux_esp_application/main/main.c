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
#include "max31865.h"
#include "mtrWebBindings.h"
#include "mainDefs.h"
#include "nvs_flash.h"
#include "motorDriver_PCA9685.h"
#include "dummyFb.h"

#define TAG "MAIN"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_ERROR;

static int logLvl = ESP_LOG_DEBUG;

#define MAX31865_CLK_PIN        18
#define MAX31865_MISO_PIN       19
#define MAX31865_MOSI_PIN       23
#define MAX31865_CS_1_PIN       26
#define MAX31865_DRDY_PIN       25

static max31865_cfg_t maxCfg = {
    .filter         = MAX31865_FILTER_50HZ,
    .clockSpeedHz   = MAX31865_MAX_CLK_HZ,
    .csPin          = MAX31865_CS_1_PIN,
    .misoPin        = MAX31865_MISO_PIN,
    .mosiPin        = MAX31865_MOSI_PIN,
    .sclkPin        = MAX31865_CLK_PIN,
    .wireMode       = MAX31865_2WIRE,
    .refResistor    = 4300,
    .rtdNominal     = 1000,
    .spiHost        = SPI2_HOST,
    .drdyPin        = -1,
};

#define VAL_LIM 1000

/****************************************************
 *
 * I2C Macros and Configurations
 *
 ****************************************************/
#define I2C_MAX_PORTS 2
#define I2C_1_SDA_PIN 4
#define I2C_1_SCL_PIN 17
#define I2C_2_SDA_PIN 32
#define I2C_2_SCL_PIN 33

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
        .enable = true,
    },
};

typedef struct {
    motorCtrlCtx_t  motorCtrl;
    webApp_t        webApp;
    wifiConn_t      wifi;
    bleSvc_t        bleSvc;
} espIf_t;

static espIf_t espIF;
/* Driver structs */
static pca9685Dev_t *pcaDev;

/* Copy of interfaces used */
static motorIF_t *mtrIF[MAX_MOTORS];
static feedback_t *fbIF[MAX_MOTORS];

static pidLoop_t dfltPids[MAX_MOTORS] = {
    [MOTOR_1] = {
        .kp     = 0.6f,
        .ki     = 0.4f,
        .kd     = 0.5f,
        .minOut = -1.0f,
        .maxOut = 1.0f,
    },
    [MOTOR_2] = {
        .kp     = 0.1f,
        .ki     = 0.2f,
        .kd     = 0.3f,
        .minOut = -1.0f,
        .maxOut = 1.0f,
    },
};

static as5600_cfg_t as5600Cfgs[MAX_MOTORS] = {
    [MOTOR_1] = {
        .devCfg.dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .devCfg.device_address     = AS5600_ADDR,
        .devCfg.scl_speed_hz       = 400000,
        .readTimeout  = I2C_READ_TIMEOUT / portTICK_PERIOD_MS,
        .writeData[0] = ANGLE_MSB,
        .writeData[1] = ANGLE_MSB >> 8,
    },
    [MOTOR_2] = {
        .devCfg.dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .devCfg.device_address     = AS5600_ADDR,
        .devCfg.scl_speed_hz       = 400000,
        .readTimeout  = I2C_READ_TIMEOUT / portTICK_PERIOD_MS,
        .writeData[0] = ANGLE_MSB,
        .writeData[1] = ANGLE_MSB >> 8,
    },
};


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
    int32_t cntr = 0;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
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

/** Initialization function for MotorIF and Fb methods. Populate the motorCtrlCtx_t struct
 *
 *  Current Assumption:
 *  - I2C busses used are already initialized
 *  - Only setting up motors that ARE being used so far
 */
static resp_t initMotor_FbDrivers(motorCtrlCtx_t *motorCtrl)
{
    CHECK_PTR_RET_ERR(motorCtrl, "motorCtrl is not allocated");

    /* Initialize the different motor drivers and motorIF */

    // One for 4 motors
    pca9685Cfg_t pcaCfg = {
        .dbgFlag = DBG_INFO | DBG_ERROR,
        .pwmFreqHz = PCA9685_MAX_FREQ,
        .i2cBus = i2cMasterCfg[0].bus,
        .devCfg = {
            .dev_addr_length  = I2C_ADDR_BIT_LEN_7,
            .device_address   = 0x40,
            .scl_speed_hz     = 400000,
        },
    };

    pcaDev = pca9685Init(pcaCfg);
    if (!pcaDev) {
        LOG_E("PCA Driver not created");
        return RESP_ERR;
    }

    // Creating individual motorIF instances
    motorDriverPCA9685Cfg_t mtrDrvPCA[MAX_MOTORS] = {
        [MOTOR_1] = {
            .channelPos = 0,
            .channelNeg = 1,
            .dev = pcaDev,
        },
        [MOTOR_2] = {
            .channelPos = 2,
            .channelNeg = 3,
            .dev = pcaDev,
        },
    };

    for (int i = 0; i < MAX_MOTORS; i++) {
        mtrIF[i] = createMtrDriverIF_PCA9685(mtrDrvPCA[i]);
        CHECK_PTR_RET_ERR(mtrIF[i], "Error creating motorIF %d for PCA", i);

        if (i == MOTOR_1) {
            as5600Cfgs[i].i2cBus = i2cMasterCfg[1].bus;
            fbIF[i] = as5600Init(as5600Cfgs[i]);
        }
        else {
            fbIF[i] = dummyFbInit(10 + 10 * i);
        }
        CHECK_PTR_RET_ERR(fbIF[i], "Error when initializing Fb ptr %d", i);

        motorCtrl->mtrs[i].motorIF      = mtrIF[i];
        motorCtrl->mtrs[i].fb           = fbIF[i];
        motorCtrl->mtrs[i].limits.lower = 0;
        motorCtrl->mtrs[i].limits.upper = 1000;
        motorCtrl->mtrs[i].idx          = i;
        motorCtrl->mtrs[i].pid.kp       = dfltPids[i].kp;
        motorCtrl->mtrs[i].pid.ki       = dfltPids[i].ki;
        motorCtrl->mtrs[i].pid.kd       = dfltPids[i].kd;
        motorCtrl->mtrs[i].pid.minOut   = dfltPids[i].minOut;
        motorCtrl->mtrs[i].pid.maxOut   = dfltPids[i].maxOut;
    }

    motorCtrl->numMotors = MAX_MOTORS;
    motorCtrl->debugFlag = DBG_DEBUG | DBG_ERROR | DBG_INFO | DBG_WARNING;
    return RESP_OK;
}


void app_main(void)
{
    resp_t sts = RESP_OK;
    esp_log_level_set(TAG, logLvl); // Setting debug
    sts = initFlash();
    RETURN_IF_ERR_LOG(sts, "Error with Nvs Flash");

    sts = initI2CPorts();
    RETURN_IF_ERR_LOG(sts, "Error initalizing I2C Buses");

    start_ble_service(&espIF.bleSvc);

    // Create motorIF and FB
    sts = initMotor_FbDrivers(&espIF.motorCtrl);
    RETURN_IF_ERR_LOG(sts, "Done MotorCtrl init %d", sts);

    sts = motorCtrlInit(&espIF.motorCtrl);
    RETURN_IF_ERR_LOG(sts, "Done MotorCtrl init %d", sts);

    sts = wifi_conn_init(&espIF.wifi);
    RETURN_IF_ERR_LOG(sts, "Error with Wifi init");

    sts = startHttpServer(&espIF.webApp);
    RETURN_IF_ERR_LOG(sts, "Error when starting webApp");

    motorCtrlRegisterWebBindings(&espIF.webApp, &espIF.motorCtrl);

    // espIF.rtdSensor = max31865Init(maxCfg);

    // if (!espIF.rtdSensor) {
    //     LOG_E("Error when initializing RTD module");
    //     return;
    // }

    xTaskCreate(testTask, "testTask", 4096, &espIF, 10, NULL);
}
