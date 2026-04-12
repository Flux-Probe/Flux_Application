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

typedef struct {
    uint16_t     dbgFlag;

    i2cCfg_t i2cCfg;
    i2c_master_bus_handle_t i2cBus;
    i2c_master_dev_handle_t i2cDev;

    uint8_t  dataBuf[MAX_READ_SIZE];
    int32_t  rawData;
    float    angle;
    uint8_t  readCount;
    uint8_t  writeData[MAX_WRITE_SIZE];
    uint32_t i2cPort;
    uint32_t readTimeout;
} as5600PrivCfg_t;

// ───────── AS5600 ─────────
static resp_t readAS5600Raw(feedback_t *fb, float *rawVal)
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
        LOG_E("I2C read failed: %s|%d", esp_err_to_name(err), cfg->i2cPort);
        sts = RESP_ERR;
    }
    else {
        cfg->rawData = ((cfg->dataBuf[0] & 0xF) << 8) | cfg->dataBuf[1]; // 12-bit value
    }

    *rawVal = cfg->rawData;
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
    sts = readAS5600Raw(fb, &tempRaw);
    if (sts != RESP_ERR) {
        if (cfg->rawData >= 0) {
            cfg->angle = cfg->rawData * 360.0f / 4096.0f;
        }
        else {
            cfg->angle = -1.0f;
        }
        LOG_V("Reading: %.2f", cfg->angle);
        *readVal = cfg->angle;
    }
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

static resp_t i2cConfigure(as5600PrivCfg_t *cfg, as5600_cfg_t pubCfg)
{
    /*TODO: Likely throw the pins and ports into a "privCfg" struct since we
            will likely have multiple of these
    // */
    cfg->i2cCfg.masterCfg.i2c_port = pubCfg.i2cCfg.masterCfg.i2c_port;
    cfg->i2cCfg.masterCfg.sda_io_num = pubCfg.i2cCfg.masterCfg.sda_io_num;
    cfg->i2cCfg.masterCfg.scl_io_num = pubCfg.i2cCfg.masterCfg.scl_io_num;
    cfg->i2cCfg.masterCfg.clk_source = pubCfg.i2cCfg.masterCfg.clk_source;

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg->i2cCfg.masterCfg, &cfg->i2cBus));

    cfg->i2cCfg.devCfg.dev_addr_length  = pubCfg.i2cCfg.devCfg.dev_addr_length;
    cfg->i2cCfg.devCfg.device_address   = pubCfg.i2cCfg.devCfg.device_address;
    cfg->i2cCfg.devCfg.scl_speed_hz     = pubCfg.i2cCfg.devCfg.scl_speed_hz;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(cfg->i2cBus, &cfg->i2cCfg.devCfg,
                                              &cfg->i2cDev));

    // cfg->readTimeout = I2C_READ_TIMEOUT / portTICK_PERIOD_MS;
    // cfg->writeData[0] = ANGLE_MSB;
    // cfg->writeData[1] = ANGLE_MSB >> 8;

    return RESP_OK;
}


feedback_t *as5600Init(as5600_cfg_t cfg)
{
    feedback_t *as5600Fb = (feedback_t *) calloc(1, sizeof(feedback_t));

    cfg.dbgFlag = ESP_LOG_DEBUG;
    esp_log_level_set(TAG, cfg.dbgFlag); // Setting debug

    as5600PrivCfg_t *privCtx = (as5600PrivCfg_t *) calloc(1, sizeof(as5600PrivCfg_t));
    // privCtx->i2cCfg = cfg.i2cCfg;

    privCtx->writeData[0] = cfg.writeData[0];
    privCtx->writeData[1] = cfg.writeData[1];
    privCtx->readTimeout = cfg.readTimeout;
    privCtx->dbgFlag = cfg.dbgFlag;
    privCtx->i2cPort = cfg.i2cCfg.masterCfg.i2c_port;

    resp_t resp = i2cConfigure(privCtx, cfg);
    LOG_W("I2C Cfg: %d,%d", privCtx->i2cCfg.masterCfg.i2c_port, privCtx->i2cCfg.devCfg.device_address);
    RETURN_VAL_IF_ERR(resp, NULL);

    as5600Fb->privCtx = (void *)privCtx;
    as5600Fb->readRawData = readAS5600Raw;
    as5600Fb->readData    = readAS5600Deg;
    as5600Fb->resetData   = resetAS5600Angle;

    LOG_I("Iniitialized I2C Successfully");

    return as5600Fb;
}