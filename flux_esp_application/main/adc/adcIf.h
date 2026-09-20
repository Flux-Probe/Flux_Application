#ifndef _ADC_IF_H
#define _ADC_IF_H

#include "mainDefs.h"

typedef struct adcIF_s adcIF_t;

struct adcIF_s{
    resp_t (*read) (adcIF_t *adcIf, uint8_t channel, float *convVal);

    resp_t (*setConfig) (adcIF_t *adcIF_t);
    void    *privCtx;
};

#endif