#ifndef I2C_TYPES_H
#define I2C_TYPES_H

#include "driver/i2c_master.h"

typedef struct {
    i2c_master_bus_config_t masterCfg;
    i2c_device_config_t     devCfg;
} i2cCfg_t;

#endif // I2C_TYPES_H
