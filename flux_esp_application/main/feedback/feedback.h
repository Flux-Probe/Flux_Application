#ifndef _FEEDBACK_H
#define _FEEDBACK_H

#include "mainDefs.h"

typedef struct feedback_s{
    resp_t (*readRawData) (struct feedback_s *fb, float *readVal);

    /* Do any conversions we need to make the data usable here*/
    resp_t (*readData)    (struct feedback_s *fb, float *readVal);
    resp_t (*resetData)   (struct feedback_s *fb, float resetVal);

    void    *privCtx;
} feedback_t;

#endif