#ifndef _AS5600_DRIVER
#define _AS5600_DRIVER

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "feedback.h"

#define MAX_READ_SIZE 2 //Only 2 bytes reading at a time
#define MAX_WRITE_SIZE 2 //Only 2 bytes reading at a time

// ───────── DEFINES ─────────
#define AS5600_ADDR 0x36
#define ANGLE_MSB   0x0E

// TODO: Replace this #if with different configuration names.
// #if 1
#define SDA_1_PIN     18
#define SCL_1_PIN     19
#define SDA_2_PIN     26
#define SCL_2_PIN     25
#define SDA_PULLUP_EN GPIO_PULLUP_ENABLE
#define SCL_PULLUP_EN GPIO_PULLUP_ENABLE
#define I2C_CLK_SPD 400000
#define I2C_READ_TIMEOUT 1000
// #endif //if 1

typedef struct {
    i2c_master_bus_config_t masterCfg;
    i2c_device_config_t devCfg;
} i2cCfg_t;

typedef struct {
    uint16_t dbgFlag;
    i2cCfg_t i2cCfg;
    uint8_t  writeData[MAX_WRITE_SIZE];
    uint32_t i2cPort;
    uint32_t readTimeout;
} as5600_cfg_t;

feedback_t *as5600Init(as5600_cfg_t cfg);

#endif