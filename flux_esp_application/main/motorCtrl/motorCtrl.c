#include "motorCtrl.h"
#include <esp_timer.h>
#include <math.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Needed for Logging module name
#define TAG "MotorCtrl"

// Default Values
#define IN1_PIN 22
#define IN2_PIN 23

#define FREQ_25HZ 40 // 25Hz
#define FREQ_1HZ 1000
#define FREQ_2HZ 500

static gpio_config_t mtr_gpio_cfg = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1 << IN1_PIN) | (1 << IN2_PIN),
    // TODO: Do we need pull up/pull down OR Interrupt?
};

mtrCfg_t valveMtrCfg = {
    .pinHigh = IN1_PIN,
    .pinLow  = IN2_PIN,
};

// TODO: Likely move to a custom cfg file if we want like "Profiles"
mtrState_t valveMtrStateDflt = {
    .debugFlag   = ESP_LOG_ERROR | ESP_LOG_DEBUG,
    .driverState = STATE_STARTUP,
    .driveDir    = MTR_STOP,
    .cfg         = &valveMtrCfg,
    .currAngle   = A_START,
    .targetAngle = A_START,
    .prevAngle   = 0,
    .openPct     = 0,
    .angleVel    = 0,
    .loopFreq    = FREQ_2HZ, // TODO: Revert back to 25Hz
    .error       = 0,
    .rawCurrent  = 0,

    .coast = {
        .active       = false,
        .t_coast      = 0,
        .alpha        = 0.5,
        .min          = 0.2,
        .max          = 25,
        .fwdCoast     = 4,
        .revCoast     = 4,
        .kvSec        = 0.03,
        .angleAtStop  = 0,
        .dirAtStop    = MTR_STOP,
        .minCoastTime = 120.0,
        .maxCoastTime = 400.0,
    },
};

void loadDefaults(mtrState_t *mtrState)
{
    memcpy(mtrState, &valveMtrStateDflt, sizeof(mtrState_t));
}

// ───── Motor ─────
void setMotorEnable(mtrState_t *mtrState, bool enable)
{
    if (enable != mtrState->enabled) {
        // Setting either on or off, set mode and drive to OFF
        setMotorDrive(mtrState, MTR_STOP);
        mtrState->driveMode = MODE_OFF;
        LOG_I("Setting Motor to :%d", enable);
    }
    mtrState->enabled = enable;
}
void setDriveMode(mtrState_t *mtrState, mtrDriveMode_e setMode)
{

    mtrState->driveMode = setMode;
}

void setMotorDrive(mtrState_t *mtrState, mtrDriveDir_e setDrive)
{
    int highPin = LOW;
    int lowPin = LOW;
    char driveDir[6] = "";
    // if (mtrState->driveDir == setDrive) {
    //     return;
    // }
    switch (setDrive)
    {
    case MTR_STOP:
        highPin = LOW;
        lowPin = LOW;
        strcpy(driveDir,"STOP");
        break;
    case MTR_FORWARD:
        highPin = HIGH;
        lowPin = LOW;
        strcpy(driveDir,"FWD");
        break;
    case MTR_REVERSE:
        highPin = LOW;
        lowPin = HIGH;
        strcpy(driveDir,"REV");
        break;
    default:
        LOG_E("Undefined motor drive state set: %d", setDrive);
        break;
        return;
    }
    gpio_set_level(mtrState->cfg->pinHigh, highPin);
    gpio_set_level(mtrState->cfg->pinLow, lowPin);

    LOG_V("Setting to mode '%s'(Pin %d to %d| and Pin %d to %d)",
          driveDir, mtrState->cfg->pinHigh, highPin,
          mtrState->cfg->pinLow, lowPin);
    mtrState->driveDir = setDrive;
}

// ----------- Region Detection -----------
bool inRegion(float a, float s, float e)
{
    bool rslt = (s <= e)? (a >= s && a <= e):(a >= s || a <= e);
    return rslt;
}


// Based on given angle, and limits, return percent the valve is open
float pctWithin(float a, float s, float e){
  float span = (e >= s) ? (e - s):(360.0f - s + e);
  float ctr = fmodf(s + span * 0.5f + 360.0f, 360.0f);
  float d = fabsf(a - ctr);
  if (d > 180.0f) {
    d = 360.0f - d;
  }
  return 100.0f * fmaxf(0.0f, 1.0f - d / (span * 0.5f));
}

float percentOpen(float a){
    if(inRegion(a, A_START, A_END))
        return pctWithin(a, A_START, A_END);
    if(inRegion(a, B_START, B_END))
        return pctWithin(a, B_START, B_END);
    return 0.0f;
}


void setTargetPercent(mtrState_t *mtrState, float target) {
    LOG_W("set pct: %.2f", target);
    if (target < 0.0 || target > 1.0) {
        mtrState->targetAngle = NAN;
    }
    else {
        float tempAngle = A_START + ((A_END - A_START) * target);
        mtrState->targetAngle = tempAngle;
    }
}

// ----------- Coasting Logic -----------
void startCoastMeasure(mtrState_t *mtrState){
    mtrState->coast.active = true;
    mtrState->coast.t_coast = esp_timer_get_time()/1000;
    mtrState->coast.angleAtStop = mtrState->currAngle;
    mtrState->coast.dirAtStop = mtrState->driveDir;
    setMotorDrive(mtrState, MTR_STOP);
}

void updateCoastLearn(mtrState_t *mtrState){
    coastParams_t c = mtrState->coast;
    if(!c.active) {
        return;
    }

    int64_t currTime = esp_timer_get_time() / 1000;
    float dt_s = (currTime - c.t_coast) / 1000.0f;

    if(dt_s < c.minCoastTime) {
        return;
    }

    if(fabsf(mtrState->angleVel) <= c.lowVelLim || dt_s >= c.maxCoastTime){
        float delta= mtrState->currAngle - c.angleAtStop;
        float overshoot = delta;
        if (c.dirAtStop == MTR_REVERSE) {
            overshoot = -overshoot;
        }

        if(overshoot < 0) {
            overshoot = 0;
        }
        overshoot = fminf(c.max, fmaxf(c.min, overshoot));

        if(c.dirAtStop == MTR_FORWARD) {
            c.fwdCoast = (1-c.alpha) * c.fwdCoast + c.alpha * overshoot;
        }
        else {
            c.revCoast = (1-c.alpha) * c.revCoast + c.alpha * overshoot;
        }
        c.active = false;
    }
}

void readEncoder(mtrState_t *mtr)
{
    mtr->currAngle = readAS5600Deg(&mtr->encoderCfg);
    // if (mtr->driveDir == MTR_FORWARD) {
    //     mtr->currAngle = mtr->currAngle + 0.05;
    // }
    // else if (mtr->driveDir == MTR_REVERSE) {
    //     mtr->currAngle = mtr->currAngle - 0.05;
    // }

}

void targetPositionControl(mtrState_t *mtrState)
{
    // Angle is invalid. Cut the motor Add a delay so cpu can be released
    if(mtrState->currAngle < 0) {
        setMotorDrive(mtrState, MTR_STOP);
        return;
    }

    // velocity estimation
    if(isnan(mtrState->prevAngle)) {
        mtrState->angleVel = 0;
    }
    else {
        float dt_s = (mtrState->t_loop.t_start - mtrState->t_loop.t_prev)/1000.0f;
        if (dt_s > 0) {
            float angDiff = mtrState->currAngle - mtrState->prevAngle;
            mtrState->angleVel = angDiff / dt_s;
        }
    }
    mtrState->t_loop.t_prev = mtrState->t_loop.t_start;
    mtrState->prevAngle = mtrState->currAngle;

    updateCoastLearn(mtrState);

    if(isnan(mtrState->targetAngle)) {
        return;
    }

    mtrState->error = mtrState->targetAngle - mtrState->currAngle;
    float absError = fabsf(mtrState->error);

    // Low Band
    if (absError <= BAND_STOP && mtrState->driveDir != MTR_STOP) {
        startCoastMeasure(mtrState);
        return;
    }

    mtrDriveDir_e targetDir = (mtrState->error > 0)? MTR_FORWARD:
                                                        MTR_REVERSE;

    if (absError <= NEAR_DEG) {
        float stopDist = (targetDir == MTR_FORWARD)?
                            mtrState->coast.fwdCoast: mtrState->coast.revCoast;
        stopDist += BAND_STOP + mtrState->coast.kvSec * fabsf(mtrState->angleVel);
        stopDist = fminf(NEAR_DEG, fmaxf(BAND_STOP + 0.2f, stopDist));

        if (absError <= stopDist) {
            if (mtrState->driveDir != MTR_STOP) {
                startCoastMeasure(mtrState);
            }
            if (mtrState->coast.active) {
                return;
            }
        }
    }

    if(absError > BAND_START){ // simple control band
        if (targetDir != mtrState->driveDir) {
            if (mtrState->t_loop.t_start - mtrState->t_loop.t_lastDirChanged >= REV_DEAD_MS) {
                setMotorDrive(mtrState, targetDir);
                mtrState->t_loop.t_lastDirChanged = mtrState->t_loop.t_start;
            }
            else {
                setMotorDrive(mtrState, MTR_STOP);
            }
        }
        else {
            setMotorDrive(mtrState, mtrState->driveDir);
        }
    }
}

// ----------- Control Task -----------
void motorControlTask(void *arg)
{
    mtrState_t *mtrState = (mtrState_t*) arg;

    while(1){
        // TODO: Add a semaphore here to have this loop go at a certain rate

        vTaskDelay(pdMS_TO_TICKS(mtrState->loopFreq));
        // LOG_D("Start of loop");
        mtrState->t_loop.t_start = esp_timer_get_time() / 1000;
        LOG_I("%d,%d,%d", mtrState->enabled, mtrState->driveMode, mtrState->driveDir);
        // TODO: Replace this with a separate call/task to get the current angle
        // of the valve
        readEncoder(mtrState);

        if (!mtrState->enabled) {
            setMotorDrive(mtrState, MTR_STOP);
            continue;
        }

        // Control Loop Logic
        switch (mtrState->driveMode)
        {
        case MODE_OFF:
        case MODE_OPEN:
            setMotorDrive(mtrState, mtrState->driveDir);
            break;
        case MODE_POS:
            targetPositionControl(mtrState);
            break;
        default:
            LOG_W("INCORRECT DRIVE MODE");
            continue;
            break;
        }
    }
}

void motorInit(mtrState_t * mtrState)
{
    esp_err_t configResp = gpio_config(&mtr_gpio_cfg);
    mtrState->enabled = false;
    if (configResp == ESP_ERR_INVALID_ARG) {
        LOG_E("Err when configuring GPIO");
        mtrState->driverState = STATE_FAILED;
        return;
    }
    setMotorDrive(mtrState, MTR_STOP);

    resp_t sts = as5600Init(&mtrState->encoderCfg);
    if (sts == RESP_ERR) {
        mtrState->driverState = STATE_FAILED;
        LOG_E("Failed initializing Encoder Driver");
        return;
    }
    mtrState->driverState = STATE_OPERATIONAL;
    LOG_D("Completed Motor Init");
    // Add any other initialization calls here
    return;
};

/*
    TODO: Replace valveMtrState with passing parameter so
        motorCtrlMain(mtrState_t *mtrState_t);
    all part of a larger type so that BLE module, temp sensor and other stuff
    plugs into
*/
int motorCtrlMain(mtrState_t *mtrState) {
    esp_log_level_set(TAG, mtrState->debugFlag); // Setting debug
    loadDefaults(mtrState);

    LOG_D("DEFAULTS:\nDEBUG_FLAG: %x\nDRIVE_STATE: %d\nDRIVE_DIR %d\nMAX_COAST_T %.2f\nMIN_COAST_T %.2f\n",
           mtrState->debugFlag, mtrState->driverState, mtrState->driveDir,
           mtrState->coast.maxCoastTime, mtrState->coast.minCoastTime);

    LOG_D("Pins: %x", mtr_gpio_cfg.pin_bit_mask);
    motorInit(mtrState);

    LOG_I("Creating motor ctrl task");

    // // Create task for control.
    xTaskCreate(motorControlTask, "motor_ctrl", 4096, mtrState, 10, NULL);
    return 1;
}