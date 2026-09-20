#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "ads1115.h"

#include <stdlib.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "ADS1115"
#define DBG dbgFlag
uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

// ───────── Register addresses ─────────
#define REG_CONVERSION   0x00
#define REG_CONFIG       0x01
#define REG_LO_THRESH    0x02
#define REG_HI_THRESH    0x03

// ───────── Config register bit field macros ─────────
#define ADS1115_CFG_REG_OS_MASK         (0x8000)
#define ADS1115_CFG_REG_OS_SHIFT        (15)
#define ADS1115_CFG_REG_OS(x)           (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_OS_SHIFT))&ADS1115_CFG_REG_OS_MASK)
#define ADS1115_CFG_REG_MUX_MASK        (0x7000)
#define ADS1115_CFG_REG_MUX_SHIFT       (12)
#define ADS1115_CFG_REG_MUX(x)          (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_MUX_SHIFT))&ADS1115_CFG_REG_MUX_MASK)
#define ADS1115_CFG_REG_PGA_MASK        (0x0E00)
#define ADS1115_CFG_REG_PGA_SHIFT       (9)
#define ADS1115_CFG_REG_PGA(x)          (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_PGA_SHIFT))&ADS1115_CFG_REG_PGA_MASK)
#define ADS1115_CFG_REG_MODE_MASK       (0x0100)
#define ADS1115_CFG_REG_MODE_SHIFT      (8)
#define ADS1115_CFG_REG_MODE(x)         (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_MODE_SHIFT))&ADS1115_CFG_REG_MODE_MASK)
#define ADS1115_CFG_REG_DR_MASK         (0x00E0)
#define ADS1115_CFG_REG_DR_SHIFT        (5)
#define ADS1115_CFG_REG_DR(x)           (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_DR_SHIFT))&ADS1115_CFG_REG_DR_MASK)
#define ADS1115_CFG_REG_CMODE_MASK      (0x0010)
#define ADS1115_CFG_REG_CMODE_SHIFT     (4)
#define ADS1115_CFG_REG_CMODE(x)        (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_CMODE_SHIFT))&ADS1115_CFG_REG_CMODE_MASK)
#define ADS1115_CFG_REG_CPOL_MASK       (0x0008)
#define ADS1115_CFG_REG_CPOL_SHIFT      (3)
#define ADS1115_CFG_REG_CPOL(x)         (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_CPOL_SHIFT))&ADS1115_CFG_REG_CPOL_MASK)
#define ADS1115_CFG_REG_CLAT_MASK       (0x0004)
#define ADS1115_CFG_REG_CLAT_SHIFT      (2)
#define ADS1115_CFG_REG_CLAT(x)         (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_CLAT_SHIFT))&ADS1115_CFG_REG_CLAT_MASK)
#define ADS1115_CFG_REG_CQUE_MASK       (0x0003)
#define ADS1115_CFG_REG_CQUE_SHIFT      (0)
#define ADS1115_CFG_REG_CQUE(x)         (((uint32_t)(((uint32_t)(x))<<ADS1115_CFG_REG_CQUE_SHIFT))&ADS1115_CFG_REG_CQUE_MASK)

// CQUE=3 (0b11) disables the comparator output entirely
#define ADS1115_CQUE_DISABLE  3u

#define I2C_TIMEOUT_MS   100

static const float PGA_SCALE[] = {
    6.144f,  // ADS1115_PGA_6144
    4.096f,  // ADS1115_PGA_4096
    2.048f,  // ADS1115_PGA_2048
    1.024f,  // ADS1115_PGA_1024
    0.512f,  // ADS1115_PGA_512
    0.256f,  // ADS1115_PGA_256
};

// Conversion wait time (ms) per ads1115Dr_t — period + 1 ms margin
static const uint16_t DR_WAIT_MS[] = {
    126,  // ADS1115_8SPS
    64,   // ADS1115_16SPS
    32,   // ADS1115_32SPS
    16,   // ADS1115_64SPS
    9,    // ADS1115_128SPS
    5,    // ADS1115_250SPS
    3,    // ADS1115_475SPS
    2,    // ADS1115_860SPS
};

// compQue is a 2-bit field — bool is not wide enough (value 3 = disable)
typedef struct {
    bool        os;
    uint8_t     muxMode;
    uint8_t     pga;
    bool        opMode;   // true = single-shot
    ads1115Dr_t dr;
    bool        compMode;
    bool        compPol;
    bool        compLat;
    uint8_t     compQue;  // 0b11 = 3 disables comparator
} ads1115CfgReg_t;

typedef struct {
    i2c_master_bus_handle_t i2cBus;
    i2c_master_dev_handle_t i2cDev;
    float                   vRef;
    float                   rRef;
    float                   pgaScale;
    ads1115Pga_t            pgaBits; // Raw enum — ADS1115_CFG_REG_PGA macro handles the shift
    uint8_t                 alertPin;
    ads1115CfgReg_t         cfgReg;
    uint16_t                dbgFlag;
} ads1115Priv_t;

// ───────── Driver functions ─────────

static resp_t ads1115ReadRaw(adcIF_t *adcIf, uint8_t channel, float *convVal)
{
    CHECK_PTR_RET_ERR(adcIf);
    CHECK_PTR_RET_ERR(convVal);
    ads1115Priv_t *priv = (ads1115Priv_t *)adcIf->privCtx;
    CHECK_PTR_RET_ERR(priv);

    // MUX values 4-7 select AIN0-AIN3 single-ended vs GND
    uint16_t cfgWord = (uint16_t)(ADS1115_CFG_REG_OS(1)
                                | ADS1115_CFG_REG_MUX(channel)
                                | ADS1115_CFG_REG_PGA(priv->pgaBits)
                                | ADS1115_CFG_REG_MODE(priv->cfgReg.opMode)
                                | ADS1115_CFG_REG_DR(priv->cfgReg.dr)
                                | ADS1115_CFG_REG_CQUE(priv->cfgReg.compQue));

    uint8_t writeCmd[3] = {
        REG_CONFIG,
        (uint8_t)(cfgWord >> 8),
        (uint8_t)(cfgWord & 0xFF),
    };

    esp_err_t err = i2c_master_transmit(priv->i2cDev, writeCmd, sizeof(writeCmd),
                                        I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        LOG_E("Config write failed: %s", esp_err_to_name(err));
        return RESP_ERR;
    }

    vTaskDelay(pdMS_TO_TICKS(DR_WAIT_MS[priv->cfgReg.dr] * 2));

    uint8_t regPtr = REG_CONVERSION;
    uint8_t data[2];
    err = i2c_master_transmit_receive(priv->i2cDev, &regPtr, 1, data, sizeof(data),
                                      I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        LOG_E("Conversion read failed: %s", esp_err_to_name(err));
        return RESP_ERR;
    }

    int rawVal = (int16_t)((data[0] << 8) | data[1]);
    *convVal   = (float)rawVal * priv->pgaScale / 32768.0f;
    LOG_D("ch%u raw=%.5f | %d", channel, *convVal, rawVal);

    return RESP_OK;
}

static resp_t ads1115CfgAlertPin(ads1115Priv_t *privCfg)
{
    CHECK_PTR_RET_ERR(privCfg);
    uint16_t lowThreshVal   = 0x0000;
    uint16_t highThreshVal  = 0x8000;

    uint8_t writeCmd[3] = {
        REG_LO_THRESH,
        (uint8_t)(lowThreshVal >> 8),
        (uint8_t)(lowThreshVal & 0xFF),
    };
}

static resp_t i2cConfigure(ads1115Priv_t *cfg, ads1115Cfg_t pubCfg)
{
    esp_err_t err = i2c_master_bus_add_device(cfg->i2cBus, &pubCfg.devCfg,
                                             &cfg->i2cDev);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return RESP_ERR;
    }

    // Probe to confirm device is reachable before returning a valid handle
    if (i2c_master_probe(cfg->i2cBus, pubCfg.devCfg.device_address, 100) != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 not found at 0x%02X - check wiring, address pins, and pull-ups",
                 pubCfg.devCfg.device_address);
        return RESP_ERR;
    }

    return RESP_OK;
}

// ───────── Init ─────────
adcIF_t *ads1115Init(ads1115Cfg_t cfg)
{
    if (cfg.vRef <= 0.0f || cfg.rRef <= 0.0f) {
        ESP_LOGE(TAG, "vRef and rRef must be positive");
        return NULL;
    }
    if ((uint8_t)cfg.pga >= sizeof(PGA_SCALE) / sizeof(PGA_SCALE[0])) {
        ESP_LOGE(TAG, "Invalid PGA setting %d", cfg.pga);
        return NULL;
    }
    if ((uint8_t)cfg.dr >= sizeof(DR_WAIT_MS) / sizeof(DR_WAIT_MS[0])) {
        ESP_LOGE(TAG, "Invalid DR setting %d", cfg.dr);
        return NULL;
    }

    dbgFlag = cfg.dbgFlag;
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    adcIF_t *adcIF = (adcIF_t *)calloc(1, sizeof(adcIF_t));
    if (!adcIF) {
        return NULL;
    }

    ads1115Priv_t *priv = (ads1115Priv_t *)calloc(1, sizeof(ads1115Priv_t));
    if (!priv) {
        free(adcIF);
        return NULL;
    }

    priv->vRef           = cfg.vRef;
    priv->rRef           = cfg.rRef;
    priv->pgaScale       = PGA_SCALE[cfg.pga];
    priv->pgaBits        = cfg.pga; // Raw enum — macro shifts it by 9
    priv->dbgFlag        = cfg.dbgFlag;
    priv->alertPin       = cfg.alertPin;

    priv->cfgReg.opMode  = true; // Single-shot mode
    priv->cfgReg.dr      = cfg.dr;
    priv->cfgReg.compQue = ADS1115_CQUE_DISABLE; // 0b11: disable comparator
    priv->i2cBus         = cfg.i2cBus;

    if (i2cConfigure(priv, cfg) != RESP_OK) {
        LOG_W("I2C Cfg ERR: %d", cfg.devCfg.device_address);
        if (priv->i2cDev) {
            i2c_master_bus_rm_device(priv->i2cDev);
        }
        free(priv);
        free(adcIF);
        return NULL;
    }

    LOG_I("Initialized: vRef=%.2fV, rRef=%.0f Ohm, PGA=+/-%.3fV",
          priv->vRef, priv->rRef, priv->pgaScale);

    adcIF->read = ads1115ReadRaw;
    adcIF->privCtx = (void *)priv;

    return adcIF;
}