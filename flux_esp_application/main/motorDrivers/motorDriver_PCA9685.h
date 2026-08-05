
#ifndef _MOTORDRIVER_IF_PCA9685
#define _MOTORDRIVER_IF_PCA9685

#include <stdbool.h>
#include <stdint.h>
#include "motorDriverIF.h"
#include "pca9685.h"

typedef struct {
    pca9685Dev_t *dev;          // Shared PCA9685 device handle, from pca9685Init()
    uint8_t      channelPos;    // PCA9685 positive output channel
    uint8_t      channelNeg;    // PCA9685 negative output channel
} motorDriverPCA9685Cfg_t;

motorIF_t *createMtrDriverIF_PCA9685(motorDriverPCA9685Cfg_t cfg);

#endif
