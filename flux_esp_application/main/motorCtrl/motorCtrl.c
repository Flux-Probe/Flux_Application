#include "motorCtrl.h"
#include <esp_timer.h>
#include <math.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"

#include "as5600.h"
#include "dummyFb.h"

// Needed for Logging module name
#define TAG "MotorCtrl"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

motorCtrlCtx_t *mtrCtx;
SemaphoreHandle_t ctrlTaskSem;
SemaphoreHandle_t resetAngleSem;
/* Held by motorControlTask for the duration of each full pass over
 * ctx->mtrs[]. setLoopGains() takes it before writing, so a gains update
 * blocks until the task is between cycles instead of applying mid-cycle.
 */
static SemaphoreHandle_t gainsMutex;

// Default Values
#define IN1_PIN 22
#define IN2_PIN 23
#define IN3_PIN 16 //Connected to the on board LED. Need to remove
#define IN4_PIN 17
#define EN1_PIN 0
#define EN2_PIN 4

#define CHECK_MTR_IDX(idx)                  \
    if ((idx) >= MAX_MOTORS) {              \
        LOG_E("Invalid index provided");    \
        return;                             \
    }                                       \

#define CHECK_MTR_IDX_ERR(idx)              \
    if ((idx) >= MAX_MOTORS) {              \
        LOG_E("Invalid index provided");    \
        return RESP_ERR;                    \
    }                                       \

/* A wedged I2C bus can make every feedback-read/drive-write fail every cycle
 * without ever fully hanging. Latch the motor to STATE_FAILED after this many
 * successive failures rather than retrying forever against a dead bus. */
#define MAX_SUCCESSIVE_IO_FAILS 5

#define CLEAR_ERRORS(idx)                      \
    mtrCtx->mtrs[(idx)].pid.error      = 0;    \
    mtrCtx->mtrs[(idx)].pid.integral   = 0;    \
    mtrCtx->mtrs[(idx)].pid.prevError  = 0;    \

// ───── Motor ─────
void setMotorEnable(uint8_t idx, bool enable)
{
    CHECK_MTR_IDX(idx);
    motorCtx_t *motor = &mtrCtx->mtrs[idx];
    resp_t sts = motor->motorIF->enable(motor->motorIF);
    if (sts == RESP_OK) {
        motor->enabled = enable;
        /* Re-enabling is the explicit operator action that clears a fault
         * latched by mtrFailSafe() - give it a fresh start. */
        if (enable) {
            motor->mtrState       = STATE_OPERATIONAL;
            motor->fbFailCount    = 0;
            motor->driveFailCount = 0;
        }
    }
    else {
        motor->ctrlMode = MODE_OFF;
        motor->enabled  = false;
    }
}

void setDriveMode(uint8_t idx, mtrDriveMode_e setMode)
{
    CHECK_MTR_IDX(idx);
    motorCtx_t *motor = &mtrCtx->mtrs[idx];
    if (setMode == MODE_OFF) {
        setTargetPwm(idx, 0);
    }

    if (setMode != motor->ctrlMode) {
        CLEAR_ERRORS(idx);
    }

    motor->ctrlMode = setMode;
}

void setTargetPwm(uint8_t idx, float setDrive)
{
    CHECK_MTR_IDX(idx);
    motorCtx_t *motor = &mtrCtx->mtrs[idx];
    /* Only want to be able to change the pwm directly when in open loop.
       Otherwise reject command.

       RFI: Possibly overwrite and force to openLoop?
    */
    motor->driveSetpoint = setDrive;
}

void setTargetPos(uint8_t idx, float targetPos)
{
    CHECK_MTR_IDX(idx);
    motorCtx_t *motor = &mtrCtx->mtrs[idx];
    LIM_VAL(targetPos, motor->limits.upper, motor->limits.lower);
    motor->posSetpoint = targetPos;
}

void resetMotorAngle(void)
{
    xSemaphoreGive(resetAngleSem);
}

resp_t setLoopGains(uint8_t idx, pidLoop_t gains)
{
    CHECK_MTR_IDX_ERR(idx);

    /* Blocks until motorControlTask releases gainsMutex, i.e. until it
     * finishes its current pass over all motors. */
    BaseType_t semResp = xSemaphoreTake(gainsMutex, pdMS_TO_TICKS(500));
    if (semResp != pdTRUE) {
        LOG_E("Error when trying to set Loop Gains for mtr %d", idx);
        return RESP_ERR;
    }

    mtrCtx->mtrs[idx].pid.kp = gains.kp;
    mtrCtx->mtrs[idx].pid.ki = gains.ki;
    mtrCtx->mtrs[idx].pid.kd = gains.kd;
    mtrCtx->mtrs[idx].pid.minOut = gains.minOut;
    mtrCtx->mtrs[idx].pid.maxOut = gains.maxOut;
    CLEAR_ERRORS(idx);

    xSemaphoreGive(gainsMutex);
    return RESP_OK;
}

void setVerboseLogs(bool flag)
{
    mtrCtx->verbLogs = flag;
}

static float pidUpdate(pidLoop_t *pid, float target, float curr, float dt)
{
    pid->error = target - curr;
    pid->integral += pid->error * (dt / 1000);

    float output = (pid->kp * pid->error) + (pid->ki * pid->integral);
    /* Clamp the PWM output depending on the motor */
    LIM_VAL(output, pid->maxOut, pid->minOut);
    return output;
}

static void positionControlLoop(motorCtx_t *mtr)
{
    CHECK_PTR_RET(mtr);
    mtr->driveCmd = pidUpdate(&mtr->pid, mtr->posSetpoint, mtr->position, FREQ_125HZ);
}

/* Latches a motor to a safe, inert state after too many successive I/O
 * failures rather than continuing to drive/read a dead bus every cycle.
 * Cleared only by an explicit setMotorEnable(idx, true) call. */
static void mtrFailSafe(motorCtx_t *mtr, const char *reason)
{
    if (mtr->mtrState == STATE_FAILED) {
        return; // already latched
    }
    LOG_E("Motor %d: %u successive %s failures - forcing STATE_FAILED",
          mtr->idx, MAX_SUCCESSIVE_IO_FAILS, reason);
    mtr->mtrState = STATE_FAILED;
    mtr->ctrlMode = MODE_OFF;
    mtr->enabled  = false;
    mtr->driveCmd = 0.0f;
}

void resetAngleTask(void *arg)
{
    motorCtrlCtx_t *ctx = (motorCtrlCtx_t *) arg;
    while (1) {
        xSemaphoreTake(resetAngleSem, portMAX_DELAY);

        motorCtx_t *mtr = &ctx->mtrs[0];
        setTargetPwm(0, 0.0f);
        setDriveMode(0, MODE_OPEN);
        vTaskDelay(pdMS_TO_TICKS(200));
        mtr->fb->resetData(mtr->fb, 0);
        LOG_W("Reset Pin #1");
        vTaskDelay(pdMS_TO_TICKS(200));
        float currPos = mtr->position;

        setTargetPwm(0, -0.2);

        LOG_W("Starting Drive");
        while (abs(mtr->position - currPos) < 25) {
            vTaskDelay(pdMS_TO_TICKS(500));
            LOG_W("Still driving %.2f | %.2f", mtr->position, currPos);
        }
        setTargetPwm(0, 0.0);


        LOG_W("Done driving");

        mtr->fb->resetData(mtr->fb, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

        LOG_W("Reset Pin #2");

    }
}


// ----------- Control Task -----------
void motorControlTask(void *arg)
{
    CHECK_PTR_RET(arg);
    motorCtrlCtx_t *ctx = (motorCtrlCtx_t *) arg;
    CHECK_PTR_RET(ctx);

    LOG_I("Params %.2f | %.2f | %.2f | %.2f", ctx->mtrs[0].limits.lower, ctx->mtrs[0].limits.upper,
                                              ctx->mtrs[1].limits.lower, ctx->mtrs[1].limits.upper);

    uint32_t cntr = 0;

    while(1){
        cntr++;
        uint32_t preSemTime = esp_timer_get_time();
        xSemaphoreTake(ctrlTaskSem, pdMS_TO_TICKS(FREQ_125HZ));
        /* Held for the whole pass below so setLoopGains() can't land a
         * partial update in the middle of iterating the motors. */
        uint32_t preLoopTimer = esp_timer_get_time();
        ctx->metrics.semWaitTick = preLoopTimer - preSemTime;

        xSemaphoreTake(gainsMutex, portMAX_DELAY);
        preLoopTimer = esp_timer_get_time();
        ctx->metrics.gainsSemTick = preLoopTimer - preSemTime;
        for (int idx = 0; idx < ctx->numMotors; idx++) {
            /* RFI: Add a field in motorCtx to decimate the speed at which each
               motor is updated. */
            motorCtx_t *mtr = &ctx->mtrs[idx];
            // RFI: Check the status of the motor here


            resp_t sts = RESP_OK;
            sts = mtr->fb->readData(mtr->fb, &mtr->position);

            if (sts != RESP_OK) {
                mtr->fbFailCount++;
                LOG_W("ERR reading data from fb (%u/%u)", mtr->fbFailCount, MAX_SUCCESSIVE_IO_FAILS);
                if (mtr->fbFailCount >= MAX_SUCCESSIVE_IO_FAILS) {
                    mtrFailSafe(mtr, "feedback-read");
                }
            }
            else {
                mtr->fbFailCount = 0;
            }

            switch (mtr->ctrlMode)
            {
            case MODE_POS:
                mtr->ctrlLoop(mtr);
                break;
            case MODE_OPEN:
                mtr->driveCmd = mtr->driveSetpoint;
                break;
            case MODE_OFF:
                mtr->driveCmd = 0.0f;
                break;
            default:
                LOG_E("Invalid command mode. %d", mtr->ctrlMode);
                break;
            }

            if (!mtr->enabled) {
                mtr->ctrlMode = MODE_OFF;
                mtr->driveCmd = 0.0f;
            }

            resp_t driveSts = mtr->motorIF->setDrive(mtr->motorIF, mtr->driveCmd);
            if (driveSts != RESP_OK) {
                mtr->driveFailCount++;
                LOG_W("ERR writing drive cmd (%u/%u)", mtr->driveFailCount, MAX_SUCCESSIVE_IO_FAILS);
                if (mtr->driveFailCount >= MAX_SUCCESSIVE_IO_FAILS) {
                    mtrFailSafe(mtr, "drive-write");
                }
            }
            else {
                mtr->driveFailCount = 0;
            }
        }

        xSemaphoreGive(gainsMutex);
        ctx->metrics.overallLoopTick = esp_timer_get_time() - preLoopTimer;

        if (ctx->verbLogs && (cntr % 4 == 0)) {
            LOG_I("%d | %d | %d", ctx->metrics.overallLoopTick, ctx->metrics.semWaitTick, ctx->metrics.gainsSemTick);
        }
    }
}

resp_t motorCtrlInit(motorCtrlCtx_t *mtrCtrl)
{
    CHECK_PTR_RET_ERR(mtrCtrl);
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Setting debug

    for (int i = 0; i < MAX_MOTORS; i++) {
        if (!mtrCtrl->mtrs[i].motorIF) {
            LOG_E("MotorIF for motor %d is not populated", i);
            return RESP_ERR;
        }
        if (!mtrCtrl->mtrs[i].fb) {
            LOG_E("fbIF for motor %d is not populated", i);
            return RESP_ERR;
        }

        mtrCtrl->mtrs[i].mtrState = STATE_OPERATIONAL;
        mtrCtrl->mtrs[i].ctrlLoop = positionControlLoop;
    }

    mtrCtx = mtrCtrl;

    ctrlTaskSem   = xSemaphoreCreateBinary();
    resetAngleSem = xSemaphoreCreateBinary();

    if (ctrlTaskSem == NULL || resetAngleSem == NULL) {
        LOG_E("Error creating motorCtrl semaphore");
        return RESP_ERR;
    }

    gainsMutex = xSemaphoreCreateMutex();

    if (gainsMutex == NULL) {
        LOG_E("Error creating gains mutex");
        return RESP_ERR;
    }

    // Create task for control.
    LOG_I("Creating motor ctrl task");
    xTaskCreate(motorControlTask, "motor_ctrl", 4096, mtrCtx, 10, NULL);
    xTaskCreate(resetAngleTask, "resetAngleTask", 4096, mtrCtx, 10, NULL);
    return RESP_OK;
}