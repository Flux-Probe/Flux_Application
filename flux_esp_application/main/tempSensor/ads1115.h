#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>
#include "i2cTypes.h"
#include "tempSensor.h"

// I2C address options (determined by ADDR pin connection)
#define ADS1115_ADDR_GND  0x48  // ADDR → GND
#define ADS1115_ADDR_VDD  0x49  // ADDR → VDD
#define ADS1115_ADDR_SDA  0x4A  // ADDR → SDA
#define ADS1115_ADDR_SCL  0x4B  // ADDR → SCL

typedef enum {
    ADS1115_PGA_6144 = 0,  // ±6.144 V — 187.5 µV/LSB
    ADS1115_PGA_4096 = 1,  // ±4.096 V — 125   µV/LSB
    ADS1115_PGA_2048 = 2,  // ±2.048 V — 62.5  µV/LSB (suits 3.3 V supply)
    ADS1115_PGA_1024 = 3,  // ±1.024 V — 31.25 µV/LSB
    ADS1115_PGA_512  = 4,  // ±0.512 V — 15.63 µV/LSB
    ADS1115_PGA_256  = 5,  // ±0.256 V — 7.813 µV/LSB
} ads1115Pga_t;

typedef enum {
    ADS1115_8SPS,
    ADS1115_16SPS,
    ADS1115_32SPS,
    ADS1115_64SPS,
    ADS1115_128SPS,
    ADS1115_250SPS,
    ADS1115_475SPS,
    ADS1115_860SPS,
} ads1115Dr_t;

typedef enum {
    ADS1115_ADDR_0 = 0x48,
    ADS1115_ADDR_1 = 0x49,
    ADS1115_ADDR_2 = 0x4A,
    ADS1115_ADDR_3 = 0x4B,
} ads1115Addr_t;


typedef struct {
    i2cCfg_t      i2cCfg;       // I2C bus and device config (pins, port, address)
    uint8_t       numChannels;  // Active single-ended channels: 1–4 (AIN0…AINn-1)
    float         vRef;         // Supply voltage across the voltage divider (V), e.g. 3.3f
    float         rRef;         // Fixed reference resistor value (Ω), e.g. 1000.0f
    ads1115Pga_t  pga;          // PGA gain setting
    ads1115Dr_t   dr;           // Data Rate setting
    uint8_t       alertPin;
    uint16_t      dbgFlag;
} ads1115Cfg_t;

// Returns NULL on failure
tempSensor_t *ads1115Init(ads1115Cfg_t cfg);

#endif // ADS1115_H
