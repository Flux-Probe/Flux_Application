
#ifndef _MOTORCTRL
#define _MOTORCTRL
#include <stdbool.h>
#include <stdint.h>
#include "as5600.h"
#include "mainDefs.h"

typedef enum {
    MTR_STOP,
    MTR_FORWARD,
    MTR_REVERSE,
} mtrDriveDir_e;

typedef enum {
    STATE_STARTUP,
    STATE_BIT,
    STATE_OPERATIONAL,
    STATE_FAILED,
    STATE_MAINTENANCE,
} mtrDriveState_e;

typedef struct {
    bool active;
    int64_t t_coast;
    float alpha;
    float min;          // Min overshoot
    float max;
    float fwdCoast;
    float revCoast;
    float kvSec;
    float angleAtStop;
    float lowVelLim;
    float minCoastTime;
    float maxCoastTime;
    mtrDriveDir_e dirAtStop;
} coastParams_t;

#define HIGH 1
#define LOW  0

// ───── Control logic ─────
static const float A_START=113.0f, A_END=255.2f, B_START=282.1f, B_END=83.7f;
static const float BAND_START=2.0f, BAND_STOP=0.9f, NEAR_DEG=15.0f;
static const uint32_t CTL_DT_MS=40, REV_DEAD_MS=200;

// typedef struct {
//     float k,
// }

typedef struct {
    uint32_t pinHigh;   // Set pin High when moving forward, Low when reverse
    uint32_t pinLow;    // Set pin Low when moving forward, High when reverse
} mtrCfg_t;

typedef struct {
    uint16_t        debugFlag;
    mtrDriveState_e driverState;
    mtrDriveDir_e   driveDir;
    mtrCfg_t *cfg;

    // Loop Parameters
    float currAngle;
    float targetAngle;
    float prevAngle;
    float openPct;
    float angleVel;
    float currPctopen;

    int32_t rawCurrent; //Not used yet
    int32_t loopFreq;
    float error;

    // Coast Logic Params
    coastParams_t coast;

    /*
        TODO: Use eventually to keep track of the running state of
        the pins level. Use gpio_get_level().
        For now, just update after we set the pins as long as they are
        successful
    */
    int currPinHigh;
    int currPinLow;

    // Replace this with the specific encoder driver used
    as5600_cfg_t encoderCfg;
} mtrState_t;


void motorInit(mtrState_t* mtrState);

void setMotorDrive(mtrState_t *mtrState, mtrDriveDir_e setDrive);

float percent_open(float angle);
void set_target_percent(int pct);
void motor_control_task(void* arg);

int motorCtrlMain(void);


#endif