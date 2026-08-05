
#ifndef _PCA9685_H
#define _PCA9685_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "mainDefs.h"

// I2C address: base 0x40, shifted by the A0-A5 address pins (0x40-0x7F)
#define PCA9685_ADDR_DEFAULT 0x40
#define PCA9685_MAX_FREQ     1526
#define PCA9685_MIN_FREQ     24
#define PCA9685_MAX_CHANNELS 16
#define PCA9685_TICK_MAX     4095  // 12-bit on/off counter, 0-4095

typedef struct {
    i2c_master_bus_handle_t i2cBus;
    i2c_device_config_t     devCfg;     // I2C device config (address, speed)
    uint16_t                pwmFreqHz;  // Output PWM frequency, e.g. 50 for standard RC servos
    uint16_t                dbgFlag;
} pca9685Cfg_t;

typedef struct pca9685Dev_s pca9685Dev_t; // opaque, shared across all channels on one chip

// Returns NULL on failure
pca9685Dev_t *pca9685Init(pca9685Cfg_t cfg);

// Convenience: on=0, off=ticks. ticks is clamped to PCA9685_TICK_MAX.
resp_t pca9685SetChannelDuty(pca9685Dev_t *dev, uint8_t channel, uint16_t ticks);

resp_t pca9685SetFreq(pca9685Dev_t *dev, uint16_t freqHz);

// Chip-wide sleep/wake - affects every channel on this device
resp_t pca9685Sleep(pca9685Dev_t *dev);
resp_t pca9685Wake(pca9685Dev_t *dev);

#endif // _PCA9685_H
