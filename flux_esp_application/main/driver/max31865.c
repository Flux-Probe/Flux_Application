/*
 * Driver for the MAX31865 RTD-to-digital converter (e.g. MAX31865PMB1 Pmod).
 *
 * One-shot conversions are used instead of the chip's continuous auto-conversion
 * mode so VBIAS is only enabled for the duration of a read, reducing RTD
 * self-heating.
 */
#include "max31865.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#define TAG "MAX31865"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

#define MAX31865_XFER_BUF_LEN      4 // Largest burst read used by this driver is 2 bytes (RTD MSB/LSB)

#define MAX31865_VBIAS_SETTLE_MS   10
#define MAX31865_CONV_TIME_60HZ_MS 55
#define MAX31865_CONV_TIME_50HZ_MS 65
#define MAX31865_DRDY_TIMEOUT_MS   100

#define MAX31865_DEFAULT_CLK_HZ    1000000

// Callendar-Van Dusen coefficients (IEC 60751)
#define MAX31865_RTD_A  3.9083e-3f
#define MAX31865_RTD_B  -5.775e-7f


// ───────── Low level SPI / register access ─────────
static resp_t writeReg(max31865_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t txBuf[2] = { reg | MAX31865_WRITE_BIT, val };
    spi_transaction_t tx = {
        .length = 8 * sizeof(txBuf),
        .tx_buffer = txBuf,
    };

    esp_err_t err = spi_device_transmit(dev->spiDev, &tx);
    if (err != ESP_OK) {
        LOG_E("SPI write to reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return RESP_ERR;
    }
    return RESP_OK;
}

static resp_t readRegs(max31865_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t txBuf[MAX31865_XFER_BUF_LEN] = {0};
    uint8_t rxBuf[MAX31865_XFER_BUF_LEN] = {0};
    txBuf[0] = reg & ~MAX31865_WRITE_BIT;

    spi_transaction_t tx = {
        .length = 8 * (1 + len),
        .tx_buffer = txBuf,
        .rx_buffer = rxBuf,
    };

    esp_err_t err = spi_device_transmit(dev->spiDev, &tx);
    if (err != ESP_OK) {
        LOG_E("SPI read from reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return RESP_ERR;
    }

    memcpy(buf, &rxBuf[1], len);
    return RESP_OK;
}

static resp_t spiConfigure(max31865_t *dev, max31865_cfg_t cfg)
{
    spi_bus_config_t busCfg = {
        .mosi_io_num = cfg.mosiPin,
        .miso_io_num = cfg.misoPin,
        .sclk_io_num = cfg.sclkPin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX31865_XFER_BUF_LEN,
    };

    esp_err_t err = spi_bus_initialize(cfg.spiHost, &busCfg, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // INVALID_STATE == bus already shared/initialized
        LOG_E("SPI bus init failed: %s", esp_err_to_name(err));
        return RESP_ERR;
    }

    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = (cfg.clockSpeedHz > 0) ? cfg.clockSpeedHz : MAX31865_DEFAULT_CLK_HZ,
        .mode = MAX31865_SPI_MODE,
        .spics_io_num = cfg.csPin,
        .queue_size = 1,
    };

    err = spi_bus_add_device(cfg.spiHost, &devCfg, &dev->spiDev);
    if (err != ESP_OK) {
        LOG_E("SPI add device failed: %s", esp_err_to_name(err));
        return RESP_ERR;
    }

    return RESP_OK;
}

// ───────── Conversion sequencing ─────────
static resp_t waitForConversion(max31865_t *dev)
{
    if (dev->drdyPin >= 0) {
        TickType_t start = xTaskGetTickCount();
        while (gpio_get_level(dev->drdyPin) != 0) {
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(MAX31865_DRDY_TIMEOUT_MS)) {
                LOG_E("Timed out waiting for DRDY");
                return RESP_ERR;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return RESP_OK;
    }

    uint32_t convMs = (dev->filter == MAX31865_FILTER_50HZ) ?
                        MAX31865_CONV_TIME_50HZ_MS : MAX31865_CONV_TIME_60HZ_MS;
    vTaskDelay(pdMS_TO_TICKS(convMs));
    return RESP_OK;
}

static resp_t triggerOneShot(max31865_t *dev)
{
    resp_t sts = writeReg(dev, MAX31865_REG_CONFIG, dev->configReg | MAX31865_CFG_VBIAS);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Failed to enable VBIAS");

    vTaskDelay(pdMS_TO_TICKS(MAX31865_VBIAS_SETTLE_MS));

    sts = writeReg(dev, MAX31865_REG_CONFIG, dev->configReg | MAX31865_CFG_VBIAS | MAX31865_CFG_1SHOT);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Failed to trigger 1-shot conversion");

    sts = waitForConversion(dev);
    return sts;
}

// ───────── Public API ─────────
resp_t max31865ReadRtdRaw(max31865_t *dev, uint16_t *rawVal, bool *fault)
{
    CHECK_PTR_RET_ERR(dev);
    CHECK_PTR_RET_ERR(rawVal);

    resp_t sts = triggerOneShot(dev);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Failed to trigger conversion");

    uint8_t buf[2];
    sts = readRegs(dev, MAX31865_REG_RTD_MSB, buf, sizeof(buf));
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Failed to read RTD registers");

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    if (fault) {
        *fault = raw & 0x1; // LSB of RTD LSB register is the fault bit
    }
    *rawVal = raw >> 1; // Remaining 15 bits are the RTD conversion result

    return RESP_OK;
}

resp_t max31865ReadResistance(max31865_t *dev, float *resistanceOhms)
{
    CHECK_PTR_RET_ERR(dev);
    CHECK_PTR_RET_ERR(resistanceOhms);

    uint16_t raw;
    bool fault = false;
    resp_t sts = max31865ReadRtdRaw(dev, &raw, &fault);
    RETURN_VAL_IF_ERR(sts, sts);

    *resistanceOhms = ((float)raw * dev->refResistor) / 32768.0f;

    if (fault) {
        LOG_W("RTD fault bit set during read: raw=%u (%.1f/32768), %.2f ohm, see max31865ReadFaultStatus()",
              raw, raw / 32768.0f, *resistanceOhms);
        return RESP_ERR;
    }

    return RESP_OK;
}

resp_t max31865ReadTempC(max31865_t *dev, float *tempC)
{
    CHECK_PTR_RET_ERR(dev);
    CHECK_PTR_RET_ERR(tempC);

    float rtdRes;
    resp_t sts = max31865ReadResistance(dev, &rtdRes);
    RETURN_VAL_IF_ERR(sts, sts);

    // Callendar-Van Dusen, quadratic form (accurate for T >= 0C)
    float z1 = -MAX31865_RTD_A;
    float z2 = MAX31865_RTD_A * MAX31865_RTD_A - (4.0f * MAX31865_RTD_B);
    float z3 = (4.0f * MAX31865_RTD_B) / dev->rtdNominal;
    float z4 = 2.0f * MAX31865_RTD_B;

    float temp = (sqrtf(z2 + (z3 * rtdRes)) + z1) / z4;

    if (temp < 0.0f) {
        // Quadratic form loses accuracy below 0C; refine with the standard
        // IEC 60751 polynomial correction.
        float rpoly = rtdRes / dev->rtdNominal;
        temp = -242.02f + 2.2228f * rpoly;
        rpoly *= (rtdRes / dev->rtdNominal);
        temp += 2.5859e-3f * rpoly;
        rpoly *= (rtdRes / dev->rtdNominal);
        temp -= 4.8260e-6f * rpoly;
        rpoly *= (rtdRes / dev->rtdNominal);
        temp -= 2.8183e-8f * rpoly;
        rpoly *= (rtdRes / dev->rtdNominal);
        temp += 1.5243e-10f * rpoly;
    }

    *tempC = temp;
    LOG_W("Resistance: %.4f | Temp: %.4f", rtdRes, temp);
    return RESP_OK;
}

resp_t max31865ReadFaultStatus(max31865_t *dev, uint8_t *faultStatus)
{
    CHECK_PTR_RET_ERR(dev);
    CHECK_PTR_RET_ERR(faultStatus);
    return readRegs(dev, MAX31865_REG_FAULT_STATUS, faultStatus, 1);
}

resp_t max31865ClearFault(max31865_t *dev)
{
    CHECK_PTR_RET_ERR(dev);
    resp_t sts = writeReg(dev, MAX31865_REG_CONFIG, dev->configReg | MAX31865_CFG_FAULT_STAT_CLR);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Failed to clear fault status");
    return RESP_OK;
}

max31865_t *max31865Init(max31865_cfg_t cfg)
{
    max31865_t *dev = (max31865_t *) calloc(1, sizeof(max31865_t));
    if (!dev) {
        return NULL;
    }

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    dev->dbgFlag     = dbgFlag;
    dev->drdyPin     = cfg.drdyPin;
    dev->wireMode    = cfg.wireMode;
    dev->filter      = cfg.filter;
    dev->rtdNominal  = (cfg.rtdNominal > 0.0f)  ? cfg.rtdNominal  : MAX31865_DEFAULT_RTD_NOMINAL;
    dev->refResistor = (cfg.refResistor > 0.0f) ? cfg.refResistor : MAX31865_DEFAULT_REF_RESISTOR;

    resp_t sts = spiConfigure(dev, cfg);
    if (sts != RESP_OK) {
        free(dev);
        return NULL;
    }

    if (dev->drdyPin >= 0) {
        gpio_config_t drdyCfg = {
            .pin_bit_mask = 1ULL << dev->drdyPin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&drdyCfg);
    }

    dev->configReg = (cfg.wireMode == MAX31865_3WIRE) ? MAX31865_CFG_3WIRE : 0;
    dev->configReg |= (cfg.filter == MAX31865_FILTER_50HZ) ? MAX31865_CFG_FILT_50HZ : 0;

    sts = writeReg(dev, MAX31865_REG_CONFIG, dev->configReg);
    if (sts != RESP_OK) {
        LOG_E("Failed to write initial config register");
        free(dev);
        return NULL;
    }

    sts = max31865ClearFault(dev);
    if (sts != RESP_OK) {
        free(dev);
        return NULL;
    }

    LOG_I("MAX31865 Initialized: wireMode=%d, filter=%d, refR=%.1f, nominalR=%.1f",
          dev->wireMode, dev->filter, dev->refResistor, dev->rtdNominal);

    return dev;
}
