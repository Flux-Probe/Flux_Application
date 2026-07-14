#ifndef _PID_H
#define _PID_H

#include <stdbool.h>
#include <stdint.h>
#include "mainDefs.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float outMin;   // Clamp applied to both the output and the integral term (anti-windup)
    float outMax;
} pidCfg_t;

typedef struct {
    pidCfg_t cfg;

    float   integral;
    float   prevError;
    float   prevMeasurement;
    int64_t tPrevUs;
    bool    firstRun;
} pid_t;

// Zero the controller state and load tunings/limits. Call once before pidCompute().
resp_t pidInit(pid_t *pid, pidCfg_t cfg);

// Run one iteration of the controller. dt is measured internally (esp_timer) so
// callers can invoke this from any loop rate without tracking time themselves.
float pidCompute(pid_t *pid, float setpoint, float measurement);

// Clear integral/derivative history without touching tunings or limits.
// Use when re-enabling a loop or handing control back after a manual/open-loop period.
void pidReset(pid_t *pid);

void pidSetTunings(pid_t *pid, float kp, float ki, float kd);
void pidSetOutputLimits(pid_t *pid, float outMin, float outMax);

#endif //_PID_H
