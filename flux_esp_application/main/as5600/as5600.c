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

// ───────── AS5600 ─────────
resp_t readAS5600Raw(as5600_cfg_t *cfg)
{
    resp_t sts = RESP_OK;
    esp_err_t err = i2c_master_transmit_receive(cfg->i2cDev, cfg->writeData, 1,
                                                cfg->dataBuf, MAX_READ_SIZE,
                                                cfg->readTimeout);

    if (err != ESP_OK) {
        cfg->rawData = 0xFFFFFFFF; //Set finalized value to invalid. Prolly make a const
        LOG_E("I2C read failed: %s", esp_err_to_name(err));
        sts = RESP_ERR;
    }
    else {
        cfg->rawData = ((cfg->dataBuf[0] & 0xF) << 8) | cfg->dataBuf[1]; // 12-bit value
    }
    return sts;
}

float readAS5600Deg(as5600_cfg_t *cfg)
{
    // TODO: Is error handling needed to be done here?
    LOG_V("Starting Read");
    readAS5600Raw(cfg);

    if (cfg->rawData >= 0) {
        cfg->angle = cfg->rawData * 360.0f / 4096.0f;
    }
    else {
        cfg->angle = -1.0f;
    }
    LOG_V("Reading: %.2f", cfg->angle);
    return cfg->angle;
}

static resp_t i2cConfigure(as5600_cfg_t * cfg)
{
    cfg->i2cCfg.masterCfg.i2c_port = I2C_PORT;
    cfg->i2cCfg.masterCfg.sda_io_num = SDA_PIN;
    cfg->i2cCfg.masterCfg.scl_io_num = SCL_PIN;
    cfg->i2cCfg.masterCfg.clk_source = I2C_CLK_SRC_DEFAULT;

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg->i2cCfg.masterCfg, &cfg->i2cBus));

    cfg->i2cCfg.devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg->i2cCfg.devCfg.device_address = AS5600_ADDR;
    cfg->i2cCfg.devCfg.scl_speed_hz = I2C_CLK_SPD;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(cfg->i2cBus, &cfg->i2cCfg.devCfg,
                                              &cfg->i2cDev));

    cfg->readTimeout = I2C_READ_TIMEOUT / portTICK_PERIOD_MS;
    cfg->writeData[0] = ANGLE_MSB;
    cfg->writeData[1] = ANGLE_MSB >> 8;

    return RESP_OK;
}


resp_t as5600Init(as5600_cfg_t * cfg)
{
    cfg->dbgFlag = ESP_LOG_DEBUG;
    esp_log_level_set(TAG, cfg->dbgFlag); // Setting debug

    resp_t resp = i2cConfigure(cfg);
    RETURN_VAL_IF_ERR(resp, resp);

    LOG_I("Iniitialized I2C Successfully");

    return RESP_OK;
}