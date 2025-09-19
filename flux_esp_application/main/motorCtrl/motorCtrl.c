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

static gpio_config_t mtr_gpio_cfg = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1 << IN1_PIN) | (1 << IN2_PIN),
    // TODO: Do we need pull up/pull down OR Interrupt?
};

mtrCfg_t valveMtrCfg = {
    .pinHigh = IN1_PIN,
    .pinLow  = IN2_PIN,
};

mtrState_t valveMtrState = {
    .debugFlag   = ESP_LOG_ERROR | ESP_LOG_DEBUG,
    .driverState = STATE_STARTUP,
    .driveDir    = MTR_STOP,
    .cfg         = &valveMtrCfg,
    .currAngle = 0,
    .targetAngle = 0,
    .prevAngle = 0,
    .openPct = 0,
    .angleVel = 0,
    .loopFreq = FREQ_1HZ, // TODO: Revert back to 25Hz
    .error = 0,
    .rawCurrent  = 0,

    .coast = {
        .active = false,
        .t_coast = 0,
        .alpha = 0.5,
        .min = 0.2,
        .max = 25,
        .fwdCoast = 4,
        .revCoast = 4,
        .kvSec = 0.03,
        .angleAtStop = 0,
        .dirAtStop = MTR_STOP,
        .minCoastTime = 120.0,
        .maxCoastTime = 400.0,
    },
};

// ───── Motor ─────
void setMotorDrive(mtrState_t *mtrState, mtrDriveDir_e setDrive)
{
    int highPin = LOW;
    int lowPin = LOW;
    char driveDir[6] = "";
    if (mtrState->driveDir == setDrive) {
        return;
    }
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
          mtrState->cfg->pinHigh, highPin,
          mtrState->cfg->pinLow, lowPin);
    mtrState->driveDir = setDrive;
}

// ----------- Region Detection -----------
bool inRegion(float a, float s, float e)
{
    bool rslt = (s <= e)? (a >= s && a <= e):(a >= s || a <= e);
    return rslt;
}

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

// ----------- Control Task -----------
void motor_control_task(void *arg)
{
    mtrState_t *mtrState = (mtrState_t*) arg;
    int64_t t_prev  = 0;
    int64_t t_start = 0;
    int64_t t_lastDirChanged = 0;
    while(1){
        // TODO: Add a semaphore here to have this loop go at a certain rate

        vTaskDelay(pdMS_TO_TICKS(mtrState->loopFreq));
        LOG_D("Start of loop");
        t_start = esp_timer_get_time() / 1000;

        // TODO: Replace this with a separate call/task to get the current angle
        // of the valve
        mtrState->currAngle = readAS5600Deg(&mtrState->encoderCfg);

        // Angle is invalid. Cut the motor Add a delay so cpu can be released
        if(mtrState->currAngle < 0) {
            setMotorDrive(mtrState, MTR_STOP);
            continue;
        }

        // velocity estimation
        if(isnan(mtrState->prevAngle)) {
            mtrState->angleVel = 0;
        }
        else {
            float dt_s = (t_start - t_prev)/1000.0f;
            if (dt_s > 0) {
                float angDiff = mtrState->currAngle - mtrState->prevAngle;
                mtrState->angleVel = angDiff / dt_s;
            }
        }
        t_prev = t_start;
        mtrState->prevAngle = mtrState->currAngle;

        updateCoastLearn(mtrState);

        if(isnan(mtrState->targetAngle)) {
            continue;
        }

        mtrState->error = mtrState->targetAngle - mtrState->currAngle;
        float absError = fabsf(mtrState->error);

        // Low Band
        if (absError <= BAND_STOP && mtrState->driveDir != MTR_STOP) {
            startCoastMeasure(mtrState);
            continue;
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
                    continue;
                }
            }
        }

        if(absError > BAND_START){ // simple control band
            if (targetDir != mtrState->driveDir) {
                if (t_start - t_lastDirChanged >= REV_DEAD_MS) {
                    setMotorDrive(mtrState, targetDir);
                    t_lastDirChanged = t_start;
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
}


void motorInit(mtrState_t * mtrState)
{
    esp_err_t configResp = gpio_config(&mtr_gpio_cfg);

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
int motorCtrlMain(void) {
    esp_log_level_set(TAG, valveMtrState.debugFlag); // Setting debug

    motorInit(&valveMtrState);

    LOG_I("Creating motor ctrl task");

    // // Create task for control.
    xTaskCreate(motor_control_task, "motor_ctrl", 4096, &valveMtrState, 10, NULL);
    return 1;
}