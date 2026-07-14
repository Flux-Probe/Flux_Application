#ifndef _MAX31865_DRIVER
#define _MAX31865_DRIVER

#include <stdbool.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "mainDefs.h"

// ───────── Register Map ─────────
#define MAX31865_REG_CONFIG        0x00
#define MAX31865_REG_RTD_MSB       0x01
#define MAX31865_REG_RTD_LSB       0x02
#define MAX31865_REG_HFAULT_MSB    0x03
#define MAX31865_REG_HFAULT_LSB    0x04
#define MAX31865_REG_LFAULT_MSB    0x05
#define MAX31865_REG_LFAULT_LSB    0x06
#define MAX31865_REG_FAULT_STATUS  0x07

#define MAX31865_WRITE_BIT         0x80

// Configuration register (0x00) bits
#define MAX31865_CFG_VBIAS         (1 << 7)
#define MAX31865_CFG_AUTO_CONV     (1 << 6)
#define MAX31865_CFG_1SHOT         (1 << 5)
#define MAX31865_CFG_3WIRE         (1 << 4)
#define MAX31865_CFG_FAULT_STAT_CLR (1 << 1)
#define MAX31865_CFG_FILT_50HZ     (1 << 0)

// Fault status register (0x07) bits
#define MAX31865_FAULT_HIGH_THRESH  (1 << 7)
#define MAX31865_FAULT_LOW_THRESH   (1 << 6)
#define MAX31865_FAULT_REFIN_HIGH   (1 << 5)
#define MAX31865_FAULT_REFIN_LOW    (1 << 4)
#define MAX31865_FAULT_RTDIN_LOW    (1 << 3)
#define MAX31865_FAULT_OVUV         (1 << 2)

#define MAX31865_FAULT_DETECT_CYCLE_CTRL_NA     0
#define MAX31865_FAULT_DETECT_CYCLE_CTRL_AUTO   1
#define MAX31865_FAULT_DETECT_CYCLE_CTRL_MAN1   2
#define MAX31865_FAULT_DETECT_CYCLE_CTRL_MAN2   3

// SPI: mode 1 (CPOL=0, CPHA=1), max clock 5MHz, MSB first
#define MAX31865_SPI_MODE          1
#define MAX31865_MAX_CLK_HZ        5000000

// Defaults matching the MAX31865PMB1 Pmod (PT100 + 400ohm 0.1% ref resistor R7)
#define MAX31865_DEFAULT_RTD_NOMINAL   100.0f
#define MAX31865_DEFAULT_REF_RESISTOR  400.0f

typedef enum {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE,
    MAX31865_4WIRE,
} max31865WireMode_e;

typedef enum {
    MAX31865_FILTER_60HZ = 0,
    MAX31865_FILTER_50HZ,
} max31865Filter_e;

typedef struct {
    spi_host_device_t   spiHost;    // SPI2_HOST or SPI3_HOST (SPI1_HOST is reserved for flash on ESP32)
    int                  mosiPin;
    int                  misoPin;
    int                  sclkPin;
    int                  csPin;
    int                  drdyPin;   // Active-low DRDY input, or -1 to use a fixed conversion delay instead
    int                  clockSpeedHz; // 0 defaults to 1MHz (chip max is 5MHz)

    max31865WireMode_e   wireMode;
    max31865Filter_e     filter;

    float                rtdNominal;  // RTD resistance at 0C, e.g. 100.0 for PT100. 0 defaults to PT100
    float                refResistor; // Precision reference resistor value. 0 defaults to 400.0 (Pmod R7)
} max31865_cfg_t;

typedef struct max31865_s {
    uint16_t             dbgFlag;
    spi_device_handle_t  spiDev;
    int                  drdyPin;
    max31865WireMode_e   wireMode;
    max31865Filter_e     filter;
    float                rtdNominal;
    float                refResistor;
    uint8_t              configReg; // Last config value written, minus one-shot/VBIAS bits
} max31865_t;

max31865_t *max31865Init(max31865_cfg_t cfg);

resp_t max31865ReadRtdRaw(max31865_t *dev, uint16_t *rawVal, bool *fault);
resp_t max31865ReadResistance(max31865_t *dev, float *resistanceOhms);
resp_t max31865ReadTempC(max31865_t *dev, float *tempC);

resp_t max31865ReadFaultStatus(max31865_t *dev, uint8_t *faultStatus);
resp_t max31865ClearFault(max31865_t *dev);

#endif
