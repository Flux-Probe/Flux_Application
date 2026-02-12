#include "motorCtrl.h"
#include <esp_timer.h>
#include <math.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motorDriver_L293D.h"
#include "as5600.h"

// Needed for Logging module name
#define TAG "MotorCtrl"

// Default Values
#define IN1_PIN 22
#define IN2_PIN 23
#define IN3_PIN 16 //Connected to the on board LED. Need to remove
#define IN4_PIN 17
#define EN1_PIN 0
#define EN2_PIN 4

// ───── Motor ─────
void setMotorEnable(motorCtx_t *motor, bool enable)
{
    resp_t sts = motor->motorIF->enable(motor->motorIF);
    if (sts == RESP_OK) {
        motor->enabled = enable;
    }
    else {
        motor->ctrlMode = MODE_OFF;
        motor->enabled  = false;
    }
}

// void setDriveMode(mtrState_t *mtrState, mtrDriveMode_e setMode)
// {
//     mtrState->driveMode = setMode;
// }

void setMotorDrive(motorCtx_t *motor, mtrDriveDir_e setDrive)
{
    motor->dir = setDrive;
    LOG_V("Setting to mode '%d'", setDrive);
}

// // ----------- Region Detection -----------
// bool inRegion(float a, float s, float e)
// {
//     bool rslt = (s <= e)? (a >= s && a <= e):(a >= s || a <= e);
//     return rslt;
// }


// // Based on given angle, and limits, return percent the valve is open
// float pctWithin(float a, float s, float e){
//   float span = (e >= s) ? (e - s):(360.0f - s + e);
//   float ctr = fmodf(s + span * 0.5f + 360.0f, 360.0f);
//   float d = fabsf(a - ctr);
//   if (d > 180.0f) {
//     d = 360.0f - d;
//   }
//   return 100.0f * fmaxf(0.0f, 1.0f - d / (span * 0.5f));
// }

// float percentOpen(float a){
//     if(inRegion(a, A_START, A_END))
//         return pctWithin(a, A_START, A_END);
//     if(inRegion(a, B_START, B_END))
//         return pctWithin(a, B_START, B_END);
//     return 0.0f;
// }


// void setTargetPercent(mtrState_t *mtrState, float target) {
//     LOG_W("set pct: %.2f", target);
//     if (target < 0.0 || target > 1.0) {
//         mtrState->targetAngle = NAN;
//     }
//     else {
//         float tempAngle = A_START + ((A_END - A_START) * target);
//         mtrState->targetAngle = tempAngle;
//     }
// }

// ----------- Coasting Logic -----------
// void startCoastMeasure(coastParams_t *coast){
//     coast->active = true;
//     coast->t_coast = esp_timer_get_time()/1000;
//     coast->angleAtStop = coast->currAngle;
//     coast->dirAtStop = mtrState->driveDir;
//     setMotorDrive(mtrState, MTR_STOP);
// }

// void updateCoastLearn(coastParams_t *coast){
//     CHECK_PTR_RET(coast);
//     if(!coast->active) {
//         return;
//     }

//     int64_t currTime = esp_timer_get_time() / 1000;
//     float dt_s = (currTime - coast->t_coast) / 1000.0f;

//     if(dt_s < coast->minCoastTime) {
//         return;
//     }

//     if(fabsf(coast->angleVel) <= coast->lowVelLim || dt_s >= coast->maxCoastTime){
//         float delta = coast->currAngle - coast->angleAtStop;
//         float overshoot = delta;
//         if (coast->dirAtStop == MTR_REVERSE) {
//             overshoot = -overshoot;
//         }

//         if(overshoot < 0) {
//             overshoot = 0;
//         }
//         overshoot = fminf(coast->max, fmaxf(coast->min, overshoot));

//         if(coast->dirAtStop == MTR_FORWARD) {
//             coast->fwdCoast = ((1 - coast->alpha) * coast->fwdCoast)
//                             + coast->alpha * overshoot;
//         }
//         else {
//             coast->revCoast = ((1 - coast->alpha) * coast->revCoast)
//                             + coast->alpha * overshoot;
//         }
//         coast->active = false;
//     }
// }

// void readEncoder(motorCtx_t *mtr)
// {
//     // mtr->currAngle = readAS5600Deg(&mtr->encoderCfg);
//     if (mtr->driveDir == MTR_FORWARD) {
//         mtr->currAngle = mtr->currAngle + 0.05;
//     }
//     else if (mtr->driveDir == MTR_REVERSE) {
//         mtr->currAngle = mtr->currAngle - 0.05;
//     }
// }

// void targetPositionControl(motorCtx_t *motorCtx)
// {
//     CHECK_PTR_RET_ERR(motorCtx);
//     coastParams_t *params = (coastParams_t *) motorCtx->ctrlParams;
//     CHECK_PTR_RET_ERR(params);

//     // Angle is invalid. Cut the motor Add a delay so cpu can be released
//     if(motorCtx->position < 0) {
//         motorCtx->cmd = MTR_STOP;
//         // return;
//     }
//     params->currAngle = motorCtx->position;
//     params->targetAngle = motorCtx->cmd;

//     // velocity estimation
//     if(isnan(params->prevAngle)) {
//         params->angleVel = 0;
//     }
//     else {
//         float dt_s = (params->t_loop.t_start - params->t_loop.t_prev) / 1000.0f;
//         if (dt_s > 0) {
//             float angDiff = params->currAngle - params->prevAngle;
//             params->angleVel = angDiff / dt_s;
//         }
//     }
//     params->t_loop.t_prev = params->t_loop.t_start;
//     params->prevAngle = params->currAngle;

//     updateCoastLearn(params);

//     if(isnan(params->targetAngle)) {
//         return;
//     }

//     params->error = params->targetAngle - params->currAngle;
//     float absError = fabsf(params->error);

//     // Low Band
//     if (absError <= BAND_STOP && motorCtx->cmd != MTR_STOP) {
//         startCoastMeasure(mtrState);
//         return;
//     }

//     mtrDriveDir_e targetDir = (mtrState->error > 0)? MTR_FORWARD:
//                                                         MTR_REVERSE;

//     if (absError <= NEAR_DEG) {
//         float stopDist = (targetDir == MTR_FORWARD)?
//                             mtrState->coast.fwdCoast: mtrState->coast.revCoast;
//         stopDist += BAND_STOP + mtrState->coast.kvSec * fabsf(mtrState->angleVel);
//         stopDist = fminf(NEAR_DEG, fmaxf(BAND_STOP + 0.2f, stopDist));

//         if (absError <= stopDist) {
//             if (mtrState->driveDir != MTR_STOP) {
//                 startCoastMeasure(mtrState);
//             }
//             if (mtrState->coast.active) {
//                 return;
//             }
//         }
//     }

//     if(absError > BAND_START){ // simple control band
//         if (targetDir != mtrState->driveDir) {
//             if (mtrState->t_loop.t_start - mtrState->t_loop.t_lastDirChanged >= REV_DEAD_MS) {
//                 setMotorDrive(mtrState, targetDir);
//                 mtrState->t_loop.t_lastDirChanged = mtrState->t_loop.t_start;
//             }
//             else {
//                 setMotorDrive(mtrState, MTR_STOP);
//             }
//         }
//         else {
//             setMotorDrive(mtrState, mtrState->driveDir);
//         }
//     }
// }

void coastControlLoop(motorCtx_t *mtr)
{
    CHECK_PTR_RET(mtr);
    // LOG_D("Start of loop");
    resp_t sts = RESP_OK;
    // mtrState->t_loop.t_start = esp_timer_get_time() / 1000;
    // LOG_I("%d,%d,%d", mtrState->enabled, mtrState->driveMode, mtrState->driveDir);

    // sts = mtr->fb->readData(mtr->fb, &mtr->position);

    if (!mtr->enabled) {
        mtr->ctrlMode = MODE_OFF;
        mtr->cmd = MTR_STOP;
    }

    // Control Loop Logic
    switch (mtr->ctrlMode)
    {
    case MODE_OFF:
    case MODE_OPEN:
        mtr->motorIF->setDir(mtr->motorIF, mtr->dir);
        sts = mtr->motorIF->setDrive(mtr->motorIF, mtr->cmd);
        // Add error loggging?
        break;
    // case MODE_POS:
    //     targetPositionControl(mtr);
    //     break;
    default:
        LOG_W("INCORRECT DRIVE MODE"); //If this goes through this could spam
        mtr->ctrlMode = MODE_OFF;
        mtr->cmd = 0;
        sts = mtr->motorIF->setDrive(mtr->motorIF, mtr->cmd);
        break;
    }
}

// ----------- Control Task -----------
void motorControlTask(void *arg)
{
    CHECK_PTR_RET(arg);
    motorCtrlCtx_t *ctx = (motorCtrlCtx_t *) arg;
    CHECK_PTR_RET(ctx);

    while(1){
        // TODO: Add a semaphore here to have this loop go at a certain rate

        vTaskDelay(pdMS_TO_TICKS(FREQ_2HZ));
        for (int idx = 0; idx < ctx->numMotors; idx++) {
            /* RFI: Add a filed in motorCtx to decimate the speed at which each
                motor is updated.
            */
            ctx->mtrs[idx].position += 0.1;
            resp_t sts = ctx->mtrs[idx].fb->readData(ctx->mtrs[idx].fb, &ctx->mtrs[idx].position);
            if (sts != RESP_OK){
                LOG_W("ERR reading data from IDX: %d", idx);
            }
            else {
                LOG_W("READ SUCCESSFUL %d", idx);

            }
            ctx->mtrs[idx].ctrlLoop(&ctx->mtrs[idx]);
        }
        LOG_W("Ctrl Loop: %d|%d|%d|%d|=====|%d|%d|%d|%d",
                                            ctx->mtrs[0].ctrlMode, ctx->mtrs[0].enabled,
                                            ctx->mtrs[0].cmd, ctx->mtrs[0].dir,
                                            ctx->mtrs[1].ctrlMode, ctx->mtrs[1].enabled,
                                            ctx->mtrs[1].cmd, ctx->mtrs[1].dir);

    }
}


// static gpio_config_t mtr_gpio_cfg = {
//     .mode = GPIO_MODE_OUTPUT,
//     .pin_bit_mask = (1 << IN1_PIN) | (1 << IN2_PIN),
//     // TODO: Do we need pull up/pull down OR Interrupt?
// };

// mtrCfg_t valveMtrCfg = {
//     .pinHigh = IN1_PIN,
//     .pinLow  = IN2_PIN,
// };

// TODO: Likely move to a custom cfg file if we want like "Profiles"
// mtrState_t valveMtrStateDflt = {
//     .debugFlag   = ESP_LOG_ERROR | ESP_LOG_DEBUG,
//     .driverState = STATE_STARTUP,
//     .driveDir    = MTR_STOP,
//     .cfg         = &valveMtrCfg,
//     .currAngle   = A_START,
//     .targetAngle = A_START,
//     .prevAngle   = 0,
//     .openPct     = 0,
//     .angleVel    = 0,
//     .loopFreq    = FREQ_2HZ, // TODO: Revert back to 25Hz
//     .error       = 0,
//     .rawCurrent  = 0,

//     .coast = {
//         .active       = false,
//         .t_coast      = 0,
//         .alpha        = 0.5,
//         .min          = 0.2,
//         .max          = 25,
//         .fwdCoast     = 4,
//         .revCoast     = 4,
//         .kvSec        = 0.03,
//         .angleAtStop  = 0,
//         .dirAtStop    = MTR_STOP,
//         .minCoastTime = 120.0,
//         .maxCoastTime = 400.0,
//     },
// };

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

    mtrCtrlCtx->mtrs[MOTOR_1].fb = as5600Init(mtr1FbCfg);
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

    mtrCtrlCtx->mtrs[MOTOR_2].fb = mtrCtrlCtx->mtrs[MOTOR_1].fb;

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

    mtrCtrlCtx->mtrs[MOTOR_2].fb = as5600Init(mtr2FbCfg);
    CHECK_PTR_RET_ERR(mtrCtrlCtx->mtrs[MOTOR_2].fb, "Error when initializing Feedback for mtr 2");

    /*===== END of Motor 2 Init =====*/
    LOG_D("Completed Motor Init");
    // Add any other initialization calls here
    return sts;
};

/*
    TODO: Replace valveMtrState with passing parameter so
          motorCtrlMain(mtrState_t *mtrState_t);
    all part of a larger type so that BLE module, temp sensor and other stuff
    plugs into
*/
int motorCtrlInit(motorCtrlCtx_t *mtrCtrlCtx) {
    /* RFI: Here is where we are adding in the specific configuration stuff for
            The HW currently used. Eventually abstract this out to a common fn
            (a privCtx?) that will be called within here to initialize to init
            the specific drivers used.
            Ultimately this should initialize all of the common stuff in mtrCtx
            AND the specific build stuff.
    */
    CHECK_PTR_RET_ERR(mtrCtrlCtx);
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Setting debug

    mtrCtrlCtx->numMotors = 2; //TODO: Make this a macro
    resp_t sts = motorInit(mtrCtrlCtx);
    RETURN_VAL_IF_ERR_LOG(sts, sts, "Error during motor Init");


    // LOG_D("DEFAULTS:\nDEBUG_FLAG: %x\nDRIVE_STATE: %d\nDRIVE_DIR %d\nMAX_COAST_T %.2f\nMIN_COAST_T %.2f\n",
    //        mtrState->debugFlag, mtrState->driverState, mtrState->driveDir,
    //        mtrState->coast.maxCoastTime, mtrState->coast.minCoastTime);

    LOG_I("Creating motor ctrl task");

    // // Create task for control.
    xTaskCreate(motorControlTask, "motor_ctrl", 4096, mtrCtrlCtx, 10, NULL);
    return 1;
}