#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdint.h>
#include "mainDefs.h"

typedef struct tempSensor_s {
    resp_t (*readTemp) (struct tempSensor_s *ts, uint8_t channel, float *tempC);
    resp_t (*readRaw)  (struct tempSensor_s *ts, uint8_t channel, int16_t *rawVal);
    void   *privCtx;
} tempSensor_t;

#endif // TEMP_SENSOR_H
