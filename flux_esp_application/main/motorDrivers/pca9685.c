
#include "pca9685.h"
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "PCA9685_DRV"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

// ───────── Register addresses ─────────
#define REG_MODE1        0x00
#define REG_MODE2        0x01
#define REG_LED0_ON_L    0x06
#define REG_PRESCALE     0xFE

// ───────── MODE1 bits ─────────
#define MODE1_RESTART BIT(7)
#define MODE1_AI      BIT(5)  // Auto-increment register address on multi-byte writes
#define MODE1_SLEEP   BIT(4)
#define MODE1_ALLCALL BIT(0)

// ───────── MODE2 bits ─────────
#define MODE2_OUTDRV  BIT(2)  // Totem-pole output (vs open-drain)

#define OSC_CLOCK_HZ     25000000.0f
#define I2C_TIMEOUT_MS   100

struct pca9685Dev_s {
    i2c_master_bus_handle_t i2cBus;
    i2c_master_dev_handle_t i2cDev;
    uint16_t pwmFreqHz;
    uint16_t dbgFlag;
};

// ───────── Internal helpers ─────────

static resp_t pca9685WriteReg(pca9685Dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(dev->i2cDev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        LOG_E("Write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return RESP_ERR;
    }
    return RESP_OK;
}

static resp_t pca9685ReadReg(pca9685Dev_t *dev, uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit_receive(dev->i2cDev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        LOG_E("Read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return RESP_ERR;
    }
    return RESP_OK;
}

static void pca9685Destroy(pca9685Dev_t *dev)
{
    if (!dev) {
        return;
    }
    if (dev->i2cDev) {
        i2c_master_bus_rm_device(dev->i2cDev);
    }
    if (dev->i2cBus) {
        i2c_del_master_bus(dev->i2cBus);
    }
    free(dev);
}


static resp_t pca9685SetChannel(pca9685Dev_t *dev, uint8_t channel, uint16_t onTick, uint16_t offTick)
{
    CHECK_PTR_RET_ERR(dev);
    if (channel >= PCA9685_MAX_CHANNELS) {
        LOG_E("Channel %u out of range (max %u)", channel, PCA9685_MAX_CHANNELS - 1);
        return RESP_ERR;
    }

    uint8_t buf[5] = {
        (uint8_t)(REG_LED0_ON_L + (4 * channel)),
        (uint8_t)(onTick & 0xFF),
        (uint8_t)(onTick >> 8),
        (uint8_t)(offTick & 0xFF),
        (uint8_t)(offTick >> 8),
    };

    esp_err_t err = i2c_master_transmit(dev->i2cDev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        LOG_E("Set channel %u failed: %s", channel, esp_err_to_name(err));
        return RESP_ERR;
    }
    return RESP_OK;
}

// ───────── Driver functions ─────────

resp_t pca9685SetChannelDuty(pca9685Dev_t *dev, uint8_t channel, uint16_t ticks)
{
    if (ticks > PCA9685_TICK_MAX) {
        ticks = PCA9685_TICK_MAX;
    }
    return pca9685SetChannel(dev, channel, 0, ticks);
}

resp_t pca9685SetFreq(pca9685Dev_t *dev, uint16_t freqHz)
{
    CHECK_PTR_RET_ERR(dev);
    if (freqHz < 24 || freqHz > 1526) {
        LOG_E("Freq %u Hz out of range (24-1526)", freqHz);
        return RESP_ERR;
    }

    uint8_t prescale = (uint8_t)((OSC_CLOCK_HZ / (4096.0f * freqHz)) - 1.0f + 0.5f);

    uint8_t oldMode;
    RETURN_VAL_IF_ERR(pca9685ReadReg(dev, REG_MODE1, &oldMode), RESP_ERR);

    uint8_t sleepMode = (uint8_t)((oldMode & ~MODE1_RESTART) | MODE1_SLEEP);
    RETURN_VAL_IF_ERR(pca9685WriteReg(dev, REG_MODE1, sleepMode), RESP_ERR);
    RETURN_VAL_IF_ERR(pca9685WriteReg(dev, REG_PRESCALE, prescale), RESP_ERR);
    RETURN_VAL_IF_ERR(pca9685WriteReg(dev, REG_MODE1, oldMode), RESP_ERR);

    vTaskDelay(pdMS_TO_TICKS(5)); // Datasheet requires >=500us for oscillator to restart

    RETURN_VAL_IF_ERR(pca9685WriteReg(dev, REG_MODE1, (uint8_t)(oldMode | MODE1_RESTART)), RESP_ERR);

    dev->pwmFreqHz = freqHz;
    LOG_D("Set freq=%uHz, prescale=%u", freqHz, prescale);
    return RESP_OK;
}

resp_t pca9685Sleep(pca9685Dev_t *dev)
{
    CHECK_PTR_RET_ERR(dev);
    uint8_t mode1;
    RETURN_VAL_IF_ERR(pca9685ReadReg(dev, REG_MODE1, &mode1), RESP_ERR);
    return pca9685WriteReg(dev, REG_MODE1, (uint8_t)(mode1 | MODE1_SLEEP));
}

resp_t pca9685Wake(pca9685Dev_t *dev)
{
    CHECK_PTR_RET_ERR(dev);
    uint8_t mode1;
    RETURN_VAL_IF_ERR(pca9685ReadReg(dev, REG_MODE1, &mode1), RESP_ERR);
    RETURN_VAL_IF_ERR(pca9685WriteReg(dev, REG_MODE1, (uint8_t)(mode1 & ~MODE1_SLEEP)), RESP_ERR);
    vTaskDelay(pdMS_TO_TICKS(5)); // Oscillator stabilization time after wake
    return RESP_OK;
}

// ───────── Init ─────────

pca9685Dev_t *pca9685Init(pca9685Cfg_t cfg)
{
    if (cfg.pwmFreqHz == 0) {
        ESP_LOGE(TAG, "pwmFreqHz must be nonzero");
        return NULL;
    }

    dbgFlag = cfg.dbgFlag;
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    pca9685Dev_t *dev = (pca9685Dev_t *) calloc(1, sizeof(pca9685Dev_t));
    if (!dev) {
        return NULL;
    }
    dev->dbgFlag = cfg.dbgFlag;
    dev->i2cBus = cfg.i2cBus;

    esp_err_t err = i2c_master_bus_add_device(dev->i2cBus, &cfg.devCfg, &dev->i2cDev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        pca9685Destroy(dev);
        return NULL;
    }

    // Probe to confirm device is reachable before returning a valid handle
    err = i2c_master_probe(dev->i2cBus, cfg.devCfg.device_address, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 not found at 0x%02X - check wiring, address pins, and pull-ups",
                 cfg.devCfg.device_address);
        pca9685Destroy(dev);
        return NULL;
    }

    // Totem-pole outputs, respond to ALLCALL
    // Setting ALL CALL bit so that if multiple PCA chips are used, this address can be a emergency stop
    if (pca9685WriteReg(dev, REG_MODE2, MODE2_OUTDRV) != RESP_OK
        || pca9685WriteReg(dev, REG_MODE1, MODE1_ALLCALL) != RESP_OK) {
        pca9685Destroy(dev);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Wake (clear sleep) and enable register auto-increment for 4-byte channel writes
    uint8_t mode1;
    if (pca9685ReadReg(dev, REG_MODE1, &mode1) != RESP_OK) {
        pca9685Destroy(dev);
        return NULL;
    }
    mode1 = (uint8_t)((mode1 & ~MODE1_SLEEP) | MODE1_AI);
    if (pca9685WriteReg(dev, REG_MODE1, mode1) != RESP_OK) {
        pca9685Destroy(dev);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    if (pca9685SetFreq(dev, cfg.pwmFreqHz) != RESP_OK) {
        pca9685Destroy(dev);
        return NULL;
    }

    LOG_D("Initialized PCA9685 at 0x%02X, freq=%uHz",
          cfg.devCfg.device_address, cfg.pwmFreqHz);

    return dev;
}
