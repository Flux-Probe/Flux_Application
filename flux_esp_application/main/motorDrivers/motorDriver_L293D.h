
#ifndef _MOTORDRIVER_IF_L293D
#define _MOTORDRIVER_IF_L293D

#include <stdbool.h>
#include <stdint.h>
#include "motorDriverIF.h"
#include <driver/gpio.h>

typedef struct {
    gpio_config_t pinCfg;
    uint32_t      pinLow;
    uint32_t      pinHigh;
    bool          polarity;
} motorDriverL293DCfg_t;

motorIF_t *createMtrDriverIF_L293D(motorDriverL293DCfg_t cfg);

#endif
