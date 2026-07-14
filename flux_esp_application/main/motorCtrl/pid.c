#include "pid.h"
#include <math.h>
#include <esp_timer.h>
#include "mainDefs.h"

#define TAG "PID"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

resp_t pidInit(pid_t *pid, pidCfg_t cfg)
{
    CHECK_PTR_RET_ERR(pid);
    if (cfg.outMax <= cfg.outMin) {
        LOG_E("Invalid output limits: min %.2f >= max %.2f", cfg.outMin, cfg.outMax);
        return RESP_ERR;
    }

    pid->cfg = cfg;
    pidReset(pid);
    return RESP_OK;
}

float pidCompute(pid_t *pid, float setpoint, float measurement)
{
    if (pid == NULL) {
        LOG_E("INVALID PTR %d", __LINE__);
        return 0.0f;
    }

    int64_t nowUs = esp_timer_get_time();
    float dt_s = pid->firstRun ? 0.0f : (nowUs - pid->tPrevUs) / 1000000.0f;
    pid->tPrevUs = nowUs;

    float error = setpoint - measurement;

    float pTerm = pid->cfg.kp * error;

    // Integral term is clamped to the output limits as it accumulates so a
    // saturated output can't wind up past what setOutputLimits() allows.
    pid->integral += pid->cfg.ki * error * dt_s;
    pid->integral = fminf(pid->cfg.outMax, fmaxf(pid->cfg.outMin, pid->integral));
    float iTerm = pid->integral;

    // Derivative on measurement, not error, so a setpoint step doesn't cause a kick.
    float dTerm = 0.0f;
    if (!pid->firstRun && dt_s > 0.0f) {
        dTerm = -pid->cfg.kd * ((measurement - pid->prevMeasurement) / dt_s);
    }

    float output = pTerm + iTerm + dTerm;
    output = fminf(pid->cfg.outMax, fmaxf(pid->cfg.outMin, output));

    pid->prevError       = error;
    pid->prevMeasurement = measurement;
    pid->firstRun         = false;

    return output;
}

void pidReset(pid_t *pid)
{
    CHECK_PTR_RET(pid);
    pid->integral         = 0.0f;
    pid->prevError        = 0.0f;
    pid->prevMeasurement  = 0.0f;
    pid->tPrevUs          = esp_timer_get_time();
    pid->firstRun         = true;
}

void pidSetTunings(pid_t *pid, float kp, float ki, float kd)
{
    CHECK_PTR_RET(pid);
    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
}

void pidSetOutputLimits(pid_t *pid, float outMin, float outMax)
{
    CHECK_PTR_RET(pid);
    if (outMax <= outMin) {
        LOG_E("Invalid output limits: min %.2f >= max %.2f", outMin, outMax);
        return;
    }

    pid->cfg.outMin = outMin;
    pid->cfg.outMax = outMax;
    pid->integral = fminf(outMax, fmaxf(outMin, pid->integral));
}
