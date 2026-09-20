/*
 * Driver for as5600 magnetic encoders.
 *
 * TODO: If we want to test other encoders, create a common format for get and
 * set functions so it can get fed in a fn pointer to a generic struct
 *
*/
#include "as5600.h"
#include <freertos/FreeRTOS.h>

// Needed for Logging module name
#define TAG "AS5600"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

typedef struct {
    uint16_t                dbgFlag;
    bool                    useAdc;
    i2c_master_bus_handle_t i2cBus;
    i2c_master_dev_handle_t i2cDev;
    uint8_t                 dataBuf[MAX_READ_SIZE];
    uint8_t                 writeData[MAX_WRITE_SIZE];
    adcIF_t                *adcIF;
    uint8_t                 channel;

    int32_t                 rawData;
    float                   angle;
    uint8_t                 readCount;
    uint32_t                readTimeout;
} as5600PrivCfg_t;

static resp_t i2cConfigure(as5600PrivCfg_t *cfg, as5600_cfg_t pubCfg)
{
    esp_err_t err = i2c_master_bus_add_device(cfg->i2cBus, &pubCfg.devCfg,
                                             &cfg->i2cDev);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return RESP_ERR;
    }

    // Probe to confirm device is reachable before returning a valid handle
    err = i2c_master_probe(cfg->i2cBus, pubCfg.devCfg.device_address, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "As5600 not found at 0x%02X - check wiring, address pins, and pull-ups",
                 pubCfg.devCfg.device_address);
        return RESP_ERR;
    }

    return RESP_OK;
}

/* A transaction that times out can leave the bus's internal state machine
 * stuck "busy", which hangs every subsequent transaction until something
 * clears it. Reset after any failure so one glitch doesn't wedge the bus
 * for good (mirrors pca9685RecoverBus() in motorDrivers/pca9685.c). */
static void as5600RecoverBus(as5600PrivCfg_t *cfg)
{
    esp_err_t err = i2c_master_bus_reset(cfg->i2cBus);
    if (err != ESP_OK) {
        LOG_E("I2C bus reset failed: %s", esp_err_to_name(err));
    }
}

static resp_t readAS5600RawI2c(feedback_t *fb, float *rawVal)
{
    CHECK_PTR_RET_ERR(fb);
    as5600PrivCfg_t *cfg = (as5600PrivCfg_t *)fb->privCtx;
    CHECK_PTR_RET_ERR(cfg);

    resp_t sts = RESP_OK;
    esp_err_t err = i2c_master_transmit_receive(cfg->i2cDev, cfg->writeData, 1,
                                                cfg->dataBuf, MAX_READ_SIZE,
                                                cfg->readTimeout);

    if (err != ESP_OK) {
        cfg->rawData = 0xFFFFFFFF; //Set finalized value to invalid. Prolly make a const
        LOG_E("I2C read failed: %s", esp_err_to_name(err));
        as5600RecoverBus(cfg);
        sts = RESP_ERR;
    }
    else {
        cfg->rawData = ((cfg->dataBuf[0] & 0xF) << 8) | cfg->dataBuf[1]; // 12-bit value
    }

    *rawVal = cfg->rawData;
    return sts;
}

static resp_t readAS5600RawAdc(feedback_t *fb, float *rawVal)
{
    CHECK_PTR_RET_ERR(fb);
    CHECK_PTR_RET_ERR(rawVal);
    as5600PrivCfg_t *cfg = (as5600PrivCfg_t *)fb->privCtx;
    CHECK_PTR_RET_ERR(cfg);
    resp_t sts         = RESP_OK;

    sts = cfg->adcIF->read(cfg->adcIF, cfg->channel, rawVal);
    if (sts == RESP_ERR) {
        LOG_E("Error returned from ADC reading");
        return sts;
    }
    return sts;
}

static resp_t readAS5600Deg(feedback_t *fb, float *readVal)
{
    CHECK_PTR_RET_ERR(fb);
    as5600PrivCfg_t *cfg = (as5600PrivCfg_t *)fb->privCtx;
    CHECK_PTR_RET_ERR(cfg);
    resp_t sts = RESP_OK;
    // TODO: Is error handling needed to be done here?
    LOG_V("Starting Read");
    float tempRaw;

    if (cfg->useAdc) {
        sts = readAS5600RawAdc(fb, &tempRaw);
    }
    else {
        sts = readAS5600RawI2c(fb, &tempRaw);
    }
    RETURN_VAL_IF_ERR(sts, RESP_ERR);

    if (tempRaw >= 0) {
        if (!cfg->useAdc) {
            cfg->angle = tempRaw * 360.0f / 4096.0f;
        }
        else {
            cfg->angle = tempRaw * 360.0f / 3.3f;
        }
    }
    else {
        cfg->angle = -1.0f;
    }

    *readVal = cfg->angle;

    return sts;
}

static resp_t resetAS5600Angle(feedback_t *fb, float resetVal)
{
    resp_t sts = RESP_OK;
    /* TODO: Use this function to set a "Starting point" for the angle.
            Probably a field in the cfg struct that will be subtracted in the
            readDeg function*/

    return sts;
}

feedback_t *as5600Init(as5600_cfg_t cfg)
{
    cfg.dbgFlag = ESP_LOG_DEBUG;
    esp_log_level_set(TAG, cfg.dbgFlag); // Setting debug

    feedback_t *as5600Fb = (feedback_t *) calloc(1, sizeof(feedback_t));
    if (!as5600Fb) {
        LOG_E("Error allocating as5600 fb ptr");
        return NULL;
    }
    as5600PrivCfg_t *privCtx = (as5600PrivCfg_t *) calloc(1, sizeof(as5600PrivCfg_t));
    if (!privCtx) {
        LOG_E("Error allocating as5600 privCtx ptr");
        free(as5600Fb);
        return NULL;
    }

    privCtx->readTimeout = cfg.readTimeout;
    privCtx->dbgFlag     = cfg.dbgFlag;
    privCtx->useAdc      = cfg.useAdc;

    if (!privCtx->useAdc) {
        privCtx->writeData[0] = cfg.writeData[0];
        privCtx->writeData[1] = cfg.writeData[1];
        privCtx->i2cBus       = cfg.i2cBus;

        if (i2cConfigure(privCtx, cfg) != RESP_OK) {
            LOG_W("I2C Cfg ERR: %d", cfg.devCfg.device_address);
            if (privCtx->i2cDev) {
                i2c_master_bus_rm_device(privCtx->i2cDev);
            }
            free(privCtx);
            free(as5600Fb);
            return NULL;
        }
        as5600Fb->readRawData = readAS5600RawI2c;
    }
    else {
        privCtx->adcIF        = cfg.adc;
        privCtx->channel      = cfg.channel;

        as5600Fb->readRawData = readAS5600RawAdc;
    }

    as5600Fb->privCtx   = (void *)privCtx;
    as5600Fb->readData  = readAS5600Deg;
    as5600Fb->resetData = resetAS5600Angle;

    LOG_I("Initialized I2C Successfully");

    return as5600Fb;
}