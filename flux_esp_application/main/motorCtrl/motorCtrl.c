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

// ----------- Control Task -----------
void motorControlTask(void *arg)
{
    CHECK_PTR_RET(arg);
    motorCtrlCtx_t *ctx = (motorCtrlCtx_t *) arg;
    CHECK_PTR_RET(ctx);

    LOG_I("Params %.2f | %.2f | %.2f | %.2f", ctx->mtrs[0].limits.lower, ctx->mtrs[0].limits.upper,
                                              ctx->mtrs[1].limits.lower, ctx->mtrs[1].limits.upper);

    while(1){
        xSemaphoreTake(ctrlTaskSem, pdMS_TO_TICKS(FREQ_125HZ));

        /* Held for the whole pass below so setLoopGains() can't land a
         * partial update in the middle of iterating the motors. */
        xSemaphoreTake(gainsMutex, portMAX_DELAY);

        for (int idx = 0; idx < ctx->numMotors; idx++) {
            /* RFI: Add a field in motorCtx to decimate the speed at which each
               motor is updated. */
            motorCtx_t *mtr = &ctx->mtrs[idx];
            // RFI: Check the status of the motor here


            resp_t sts = RESP_OK;
            sts = mtr->fb->readData(mtr->fb, &mtr->position);

            if (sts != RESP_OK){
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
    }
}

#if 0
resp_t motorInit(motorCtrlCtx_t *mtrCtrlCtx)
{
    /*===== Motor 1 initializaiton =====*/
    motorDriverL293DCfg_t mtr1Cfg = {
        .enablePin = EN1_PIN,
        .pinHigh = IN1_PIN,
        .pinLow  = IN2_PIN,
        .polarity = 1,
        .groupId = 0,
        .pwmFreqHz = 25000,
        .timerHz = 10000000
    };
    /* motorIF init*/
    mtrCtrlCtx->mtrs[MOTOR_1].motorIF = createMtrDriverIF_L293D(mtr1Cfg);
    CHECK_PTR_RET_ERR(mtrCtrlCtx->mtrs[MOTOR_1].motorIF, "Error when initializing motorIF for mtr 1");

    /*Feedback init*/
    as5600_cfg_t mtr1FbCfg;
    mtr1FbCfg.i2cCfg.masterCfg.i2c_port    = I2C_NUM_0;
    mtr1FbCfg.i2cCfg.masterCfg.sda_io_num  = SDA_1_PIN;
    mtr1FbCfg.i2cCfg.masterCfg.scl_io_num  = SCL_1_PIN;
    mtr1FbCfg.i2cCfg.masterCfg.clk_source  = I2C_CLK_SRC_DEFAULT;

    mtr1FbCfg.i2cCfg.devCfg.dev_addr_length    = I2C_ADDR_BIT_LEN_7;
    mtr1FbCfg.i2cCfg.devCfg.device_address     = AS5600_ADDR;
    mtr1FbCfg.i2cCfg.devCfg.scl_speed_hz       = I2C_CLK_SPD;

    mtr1FbCfg.readTimeout  = I2C_READ_TIMEOUT / portTICK_PERIOD_MS;
    mtr1FbCfg.writeData[0] = ANGLE_MSB;
    mtr1FbCfg.writeData[1] = ANGLE_MSB >> 8;

#if defined(USE_DUMMY_FB)
    mtrCtrlCtx->mtrs[MOTOR_1].fb = dummyFbInit(1000);
#else
    mtrCtrlCtx->mtrs[MOTOR_1].fb = as5600Init(mtr1FbCfg);
#endif
    CHECK_PTR_RET_ERR(mtrCtrlCtx->mtrs[MOTOR_1].fb, "Error when initializing Feedback for mtr 1");

    mtrCtrlCtx->mtrs[MOTOR_1].enabled = false;
    resp_t sts = mtrCtrlCtx->mtrs[MOTOR_1].motorIF->setDrive(mtrCtrlCtx->mtrs[MOTOR_1].motorIF, 0.0);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Err setting mtr 1 drive to 0")

    mtrCtrlCtx->mtrs[MOTOR_1].mtrState = STATE_OPERATIONAL;
    mtrCtrlCtx->mtrs[MOTOR_1].ctrlLoop = coastControlLoop;

    /*===== END of Motor 1 Init =====*/
    /*===== Motor 2 initializaiton =====*/
    motorDriverL293DCfg_t mtr2Cfg = {
        .enablePin = EN2_PIN,
        .pinHigh = IN3_PIN,
        .pinLow  = IN4_PIN,
        .polarity = 1,
        .groupId = 0,
        .pwmFreqHz = 25000,
        .timerHz = 10000000
    };

    /* motorIF init*/
    mtrCtrlCtx->mtrs[MOTOR_2].motorIF = createMtrDriverIF_L293D(mtr2Cfg);
    CHECK_PTR_RET_ERR(mtrCtrlCtx->mtrs[MOTOR_2].motorIF, "Error when initializing motorIF for mtr 2");
    mtrCtrlCtx->mtrs[MOTOR_2].enabled = false;

    sts = mtrCtrlCtx->mtrs[MOTOR_2].motorIF->setDrive(mtrCtrlCtx->mtrs[MOTOR_2].motorIF, 0.0);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Err setting mtr 2 drive to 0")

    mtrCtrlCtx->mtrs[MOTOR_2].mtrState = STATE_OPERATIONAL;
    mtrCtrlCtx->mtrs[MOTOR_2].ctrlLoop = coastControlLoop;

    /* Feedback init*/
    as5600_cfg_t mtr2FbCfg;
    mtr2FbCfg.i2cCfg.masterCfg.i2c_port    = I2C_NUM_1;
    mtr2FbCfg.i2cCfg.masterCfg.sda_io_num  = SDA_2_PIN;
    mtr2FbCfg.i2cCfg.masterCfg.scl_io_num  = SCL_2_PIN;
    mtr2FbCfg.i2cCfg.masterCfg.clk_source  = I2C_CLK_SRC_DEFAULT;

    mtr2FbCfg.i2cCfg.devCfg.dev_addr_length    = I2C_ADDR_BIT_LEN_7;
    mtr2FbCfg.i2cCfg.devCfg.device_address     = AS5600_ADDR;
    mtr2FbCfg.i2cCfg.devCfg.scl_speed_hz       = I2C_CLK_SPD;

    mtr2FbCfg.readTimeout  = I2C_READ_TIMEOUT / portTICK_PERIOD_MS;
    mtr2FbCfg.writeData[0] = ANGLE_MSB;
    mtr2FbCfg.writeData[1] = ANGLE_MSB >> 8;

#if defined(USE_DUMMY_FB)
    mtrCtrlCtx->mtrs[MOTOR_2].fb = dummyFbInit(1000);
#else
    mtrCtrlCtx->mtrs[MOTOR_2].fb = as5600Init(mtr2FbCfg);
#endif
    CHECK_PTR_RET_ERR(mtrCtrlCtx->mtrs[MOTOR_2].fb, "Error when initializing Feedback for mtr 2");

    /*===== END of Motor 2 Init =====*/
    LOG_D("Completed Motor Init");
    // Add any other initialization calls here
    return sts;
};
#endif

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

    ctrlTaskSem = xSemaphoreCreateBinary();

    if (ctrlTaskSem == NULL) {
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
    return RESP_OK;
}