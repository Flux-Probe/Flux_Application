
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "motorDriver_PCA9685.h"

#define TAG "PCA9685_MTR"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;



typedef struct {
    pca9685Dev_t  *dev;
    uint8_t       channelP;
    uint8_t       channelN;
    mtrDriveDir_e dir;
} pca9685MtrPrivCtx_t;

/**
 * Helper function to send drive commands to both channels one after another
 * The order matters depending on the command.
 */
static inline resp_t sendDrv(pca9685Dev_t* dev, uint8_t c1, uint8_t c2, uint16_t d1, uint16_t d2)
{
    resp_t sts = pca9685SetChannelDuty(dev, c1, d1);
    sts |= pca9685SetChannelDuty(dev, c2, d2);
    return sts;
}

static resp_t pca9685MtrEnable(motorIF_t *state)
{
    CHECK_PTR_RET_ERR(state);
    pca9685MtrPrivCtx_t *privCtx = (pca9685MtrPrivCtx_t *)state->privCtx;
    CHECK_PTR_RET_ERR(privCtx);
    return RESP_OK;
}

static resp_t pca9685MtrDisable(motorIF_t *state)
{
    CHECK_PTR_RET_ERR(state);
    pca9685MtrPrivCtx_t *privCtx = (pca9685MtrPrivCtx_t *)state->privCtx;
    CHECK_PTR_RET_ERR(privCtx);
    return sendDrv(privCtx->dev, privCtx->channelN, privCtx->channelP, 0, 0);
}

static resp_t pca9685MtrGetDrive(motorIF_t *state)
{
    CHECK_PTR_RET_ERR(state);
    return RESP_OK;
}

static resp_t pca9685MtrGetStatus(motorIF_t *state)
{
    CHECK_PTR_RET_ERR(state);
    return RESP_OK;
}

static resp_t pca9685MtrSetDrive(motorIF_t *state, float targetPwm)
{
    CHECK_PTR_RET_ERR(state);
    pca9685MtrPrivCtx_t *privCtx = (pca9685MtrPrivCtx_t *)state->privCtx;
    CHECK_PTR_RET_ERR(privCtx);
    LIM_VAL(targetPwm, 1.0, -1.0);
    resp_t sts = RESP_OK;

    uint16_t posPWM = (targetPwm > 0)? (uint16_t)((targetPwm * PCA9685_TICK_MAX) + 0.5f): 0;
    uint16_t negPWM = (targetPwm < 0)? (uint16_t)((-targetPwm * PCA9685_TICK_MAX) + 0.5f): 0;
    LOG_D("DRIVE: %.2f | %d | %d", targetPwm, posPWM, negPWM);
    /* Always setting the 0 PWM first to avoid having both on */
    if (targetPwm < 0) {
        sts = sendDrv(privCtx->dev, privCtx->channelP, privCtx->channelN, posPWM, negPWM);
    }
    else {
        sts = sendDrv(privCtx->dev, privCtx->channelN, privCtx->channelP, negPWM, posPWM);
    }
    return sts;
}

static resp_t pca9685MtrResetMotor(motorIF_t *state)
{
    CHECK_PTR_RET_ERR(state);
    pca9685MtrPrivCtx_t *privCtx = (pca9685MtrPrivCtx_t *)state->privCtx;
    CHECK_PTR_RET_ERR(privCtx);
    return sendDrv(privCtx->dev, privCtx->channelN, privCtx->channelP, 0, 0);
}

motorIF_t *createMtrDriverIF_PCA9685(motorDriverPCA9685Cfg_t cfg)
{
    if (cfg.dev == NULL) {
        LOG_E("PCA9685 device handle is NULL - call pca9685Init first");
        return NULL;
    }
    /* TODO: Add a check later within the device that sees if any 2 motorIF are
     * using the same channels. Can add, 16 bit field and set each bit for the
     * ones used
     */
    if (cfg.channelPos >= PCA9685_MAX_CHANNELS || cfg.channelNeg >= PCA9685_MAX_CHANNELS) {
        LOG_E("Channel %u | %u out of range (max %u)", cfg.channelPos, cfg.channelNeg, PCA9685_MAX_CHANNELS - 1);
        return NULL;
    }

    motorIF_t *motorIF = (motorIF_t *) calloc(1, sizeof(motorIF_t));
    if (!motorIF) {
        return NULL;
    }

    pca9685MtrPrivCtx_t *privCtx = (pca9685MtrPrivCtx_t *) calloc(1, sizeof(pca9685MtrPrivCtx_t));
    if (!privCtx) {
        free(motorIF);
        return NULL;
    }

    privCtx->dev      = cfg.dev;
    privCtx->channelP = cfg.channelPos;
    privCtx->channelN = cfg.channelNeg;

    motorIF->enable     = pca9685MtrEnable;
    motorIF->disable    = pca9685MtrDisable;
    motorIF->getDrive   = pca9685MtrGetDrive;
    motorIF->getStatus  = pca9685MtrGetStatus;
    motorIF->setDrive   = pca9685MtrSetDrive;
    motorIF->resetMotor = pca9685MtrResetMotor;
    motorIF->privCtx    = (void *) privCtx;

    // Start with the channel forced off until a drive command arrives
    if (pca9685SetChannelDuty(privCtx->dev, privCtx->channelP, 0) != RESP_OK) {
        LOG_E("Failed to initialize channel %u to off", cfg.channelPos);
        free(privCtx);
        free(motorIF);
        return NULL;
    }
    if (pca9685SetChannelDuty(privCtx->dev, privCtx->channelN, 0) != RESP_OK) {
        LOG_E("Failed to initialize channel %u to off", cfg.channelNeg);
        free(privCtx);
        free(motorIF);
        return NULL;
    }

    LOG_I("Initialized PCA9685 motorIF on channels %u | %u", cfg.channelPos, cfg.channelNeg);
    return motorIF;
}
