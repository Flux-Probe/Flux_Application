
#include <stdbool.h>
#include <stdint.h>
#include "motorDriver_L293D.h"

// PWM Headers
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_fault.h"
#include "driver/mcpwm_sync.h"
#include "driver/mcpwm_cap.h"
#include "driver/mcpwm_types.h"

#define TAG "L293D"

typedef struct {
    bool enable;
    bool mtrPolarity; //False is normal, True is inverse
    uint32_t enablePin;
    uint32_t lowPin;
    uint32_t highPin;

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t cmpa;
    mcpwm_cmpr_handle_t cmpb;
    mcpwm_gen_handle_t gena;
    mcpwm_gen_handle_t genb;
    uint32_t maxCompVal; //Max value to set comparator compare value. Based on timer cfg
} l293dPrivCtx_t;

static resp_t l293dEnable(motorIF_t *state)
{
    l293dPrivCtx_t *privCtx = (l293dPrivCtx_t *)state->privCtx;
    CHECK_PTR_RET_ERR(privCtx);
    RETURN_VAL_IF_ERR(gpio_set_level(privCtx->enablePin, HIGH), RESP_ERR);
    // privCtx->enable = true;
    return RESP_OK;
}

static resp_t l293dDisable(motorIF_t *state)
{
    l293dPrivCtx_t *privCtx = (l293dPrivCtx_t *)state->privCtx;
    CHECK_PTR_RET_ERR(privCtx);
    RETURN_VAL_IF_ERR(gpio_set_level(privCtx->enablePin, LOW), RESP_ERR);
    // privCtx->enable = true;
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

static resp_t l293dSetDir(motorIF_t *state, mtrDriveDir_e dir)
{
    /* TODO-NK: Currently only allowing driving FWD, REV, or STOP directions
                Will add PWM cycle later on. Likely will need rework
     */
    l293dPrivCtx_t *privCtx = (l293dPrivCtx_t *)state->privCtx;
    int dirA = 0;
    int dirB = 0;
    switch (dir)
    {
    case MTR_FORWARD:
        dirA = -1;
        dirB = 0;
        break;
    case MTR_REVERSE:
        dirA = 0;
        dirB = -1;
        break;
    case MTR_STOP:
        dirA = 1;
        dirB = 1;
        break;
    default:
        LOG_W("Invalid Direction, coasting");
        break;
    }
    esp_err_t pwmSts = mcpwm_generator_set_force_level(privCtx->genb, dirB, true);
    if (pwmSts != ESP_OK) {
        LOG_E("Error when setting genB");
        return RESP_ERR;
    }
    pwmSts = mcpwm_generator_set_force_level(privCtx->gena, dirA, true);
    if (pwmSts != ESP_OK) {
        LOG_E("Error when setting genA");
        return RESP_ERR;
    }

    return RESP_OK;
}

static resp_t l293dSetDrive(motorIF_t *state, uint32_t targetPwm)
{
    /* TODO-NK: Currently only allowing driving FWD, REV, or STOP directions
                Will add PWM cycle later on. Likely will need rework
     */
    l293dPrivCtx_t *privCtx = (l293dPrivCtx_t *)state->privCtx;

    if (targetPwm > privCtx->maxCompVal) {
        targetPwm = privCtx->maxCompVal;
    }

    esp_err_t pwmSts = mcpwm_comparator_set_compare_value(privCtx->cmpa, targetPwm);
    if (pwmSts != ESP_OK) {
        LOG_W("ERR Setting cmpA Speed");
        return RESP_ERR;
    }
    pwmSts = mcpwm_comparator_set_compare_value(privCtx->cmpb, targetPwm);
    if (pwmSts != ESP_OK) {
        LOG_W("ERR Setting cmpB Speed");
        return RESP_ERR;
    }

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
    motorIF->setDir     = l293dSetDir;

    /* Set up and enable the different GPIO here*/
    gpio_config_t enablePinCfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << cfg.enablePin),
    };

    esp_err_t configGpio = gpio_config(&enablePinCfg);
    if (configGpio == ESP_ERR_INVALID_ARG) {
        LOG_E("Error when configuring enable GPIO pin");
        return NULL;
    }

    l293dPrivCtx_t *privCtx;
    privCtx = (l293dPrivCtx_t *)calloc(1, sizeof(l293dPrivCtx_t));
    privCtx->enablePin   = cfg.enablePin;
    privCtx->highPin     = cfg.pinHigh;
    privCtx->lowPin      = cfg.pinLow;
    privCtx->mtrPolarity = cfg.polarity;
    privCtx->maxCompVal  = cfg.timerHz / cfg.pwmFreqHz;

    mcpwm_timer_config_t timerCfg = {
        .group_id = cfg.groupId,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = cfg.timerHz,
        .period_ticks = privCtx->maxCompVal,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    esp_err_t pwmErr = mcpwm_new_timer(&timerCfg, &privCtx->timer);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when timer Init");
        return NULL;
    }

    mcpwm_operator_config_t operatorCfg = {
        .group_id = cfg.groupId,
    };

    pwmErr = mcpwm_new_operator(&operatorCfg, &privCtx->operator);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when operator Init");
        return NULL;
    }
    pwmErr = mcpwm_operator_connect_timer(privCtx->operator, privCtx->timer);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when connecting timer to operator");
        return NULL;
    }

    mcpwm_comparator_config_t compCfg = {
        .flags.update_cmp_on_tez = true,
    };
    pwmErr = mcpwm_new_comparator(privCtx->operator, &compCfg, &privCtx->cmpa);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when creating CmpA");
        return NULL;
    }
    pwmErr = mcpwm_new_comparator(privCtx->operator, &compCfg, &privCtx->cmpb);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when creating CmpA");
        return NULL;
    }

    // Setting the starting PWM to 0
    mcpwm_comparator_set_compare_value(privCtx->cmpa, 0);
    mcpwm_comparator_set_compare_value(privCtx->cmpb, 0);

    mcpwm_generator_config_t generatorCfg = {
        .gen_gpio_num = privCtx->lowPin,
    };

    pwmErr = mcpwm_new_generator(privCtx->operator, &generatorCfg, &privCtx->gena);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when creating generator A");
        return NULL;
    }
    generatorCfg.gen_gpio_num = privCtx->highPin;
    pwmErr = mcpwm_new_generator(privCtx->operator, &generatorCfg, &privCtx->genb);
    if (pwmErr != ESP_OK) {
        LOG_E("Error when creating generator B");
        return NULL;
    }

    mcpwm_generator_set_action_on_timer_event(privCtx->gena,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(privCtx->gena,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, privCtx->cmpa, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_timer_event(privCtx->genb,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(privCtx->genb,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, privCtx->cmpb, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(privCtx->timer);
    mcpwm_timer_start_stop(privCtx->timer, MCPWM_TIMER_START_NO_STOP);

    motorIF->privCtx     = (void*) privCtx;
    return motorIF;
}