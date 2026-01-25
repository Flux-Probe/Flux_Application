
#ifndef _MOTORDRIVER_IF_L293D
#define _MOTORDRIVER_IF_L293D

#include <stdbool.h>
#include <stdint.h>
#include "motorDriverIF.h"
#include <driver/gpio.h>

typedef struct {
    uint32_t    enablePin;
    uint32_t    pinLow;
    uint32_t    pinHigh;
    bool        polarity;

    int groupId;
    uint32_t timerHz;
    uint32_t pwmFreqHz;
} motorDriverL293DCfg_t;

motorIF_t *createMtrDriverIF_L293D(motorDriverL293DCfg_t cfg);

#endif
