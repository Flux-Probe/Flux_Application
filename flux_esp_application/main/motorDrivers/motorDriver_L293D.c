
#include <stdbool.h>
#include <stdint.h>
#include "motorDriver_L293D.h"

#define TAG "L293D"

typedef struct {
    bool enable;
    bool mtrPolarity; //False is normal, True is inverse
    uint32_t lowPin;
    uint32_t highPin;
} l293dPrivCtx_t;

static resp_t l293dEnable(motorIF_t *state)
{
    return RESP_OK;
}

static resp_t l293dDisable(motorIF_t *state)
{
    return RESP_OK;
}

static resp_t l293dGetDrive(motorIF_t *state)
{
    return RESP_OK;
}

static resp_t l293dGetStatus(motorIF_t *state)
{
    return RESP_OK;
}

static resp_t l293dSetDrive(motorIF_t *state, float targetPwm)
{
    /* TODO-NK: Currently only allowing driving FWD, REV, or STOP directions
                Will add PWM cycle later on. Likely will need rework
     */
    l293dPrivCtx_t *privCtx = (l293dPrivCtx_t *)state->privCtx;
    int lowPin = LOW;
    int highPin = LOW;
    if (targetPwm > 0) { //FWD Direction
        lowPin = LOW;
        highPin = HIGH;
    }
    else if (targetPwm < 0) {
        lowPin = HIGH;
        highPin = LOW;
    }

    if (privCtx->mtrPolarity) {
        int tempPin = lowPin;
        lowPin = highPin;
        highPin = tempPin;
    }

    gpio_set_level(privCtx->highPin, highPin);
    gpio_set_level(privCtx->lowPin, lowPin);

    return RESP_OK;
}

static resp_t l293dResetMotor(motorIF_t *state)
{
    return RESP_OK;
}

motorIF_t *createMtrDriverIF_L293D(motorDriverL293DCfg_t cfg)
{
    motorIF_t *motorIF = (motorIF_t*) calloc(1, sizeof(motorIF_t));
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Setting debug

    motorIF->enable     = l293dEnable;
    motorIF->disable    = l293dDisable;
    motorIF->getDrive   = l293dGetDrive;
    motorIF->getStatus  = l293dGetStatus;
    motorIF->setDrive   = l293dSetDrive;
    motorIF->resetMotor = l293dResetMotor;

    /* Set up and enable the different GPIO here*/
    esp_err_t configGpio = gpio_config(&cfg.pinCfg);
    if (configGpio == ESP_ERR_INVALID_ARG) {
        LOG_E("Error when configuring GPIO");
        return NULL;
    }

    l293dPrivCtx_t *privCtx;
    privCtx = (l293dPrivCtx_t *)calloc(1, sizeof(l293dPrivCtx_t));
    privCtx->highPin     = cfg.pinHigh;
    privCtx->lowPin      = cfg.pinLow;
    privCtx->mtrPolarity = cfg.polarity;
    motorIF->privCtx     = (void*) privCtx;

    return motorIF;
}