
#ifndef _MOTORCTRL
#define _MOTORCTRL
#include <stdbool.h>
#include <stdint.h>
#include "as5600.h"
#include "mainDefs.h"
#include "motorDriverIF.h"
#include "feedback.h"

#define FREQ_25HZ 40
#define FREQ_1HZ  1000
#define FREQ_2HZ  500

enum {
    MOTOR_1,
    MOTOR_2,

    MAX_MOTORS,
};

typedef enum {
    STATE_STARTUP,
    STATE_BIT,
    STATE_OPERATIONAL,
    STATE_FAILED,
    STATE_MAINTENANCE,
} mtrDriveState_e;

typedef enum {
    MODE_OFF,
    MODE_POS,  //Used when going in a target position
    MODE_OPEN, //Used when driving a set command
} mtrDriveMode_e;


// ───── Control logic ─────
static const float A_START=113.0f, A_END=255.2f, B_START=282.1f, B_END=83.7f;
static const float BAND_START=2.0f, BAND_STOP=0.9f, NEAR_DEG=15.0f;
static const uint32_t CTL_DT_MS=40, REV_DEAD_MS=200;

typedef struct {
    int64_t t_prev;
    int64_t t_start;
    int64_t t_lastDirChanged;
} positionControlTimeParams_t;

typedef struct {
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
    positionControlTimeParams_t t_loop;

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

typedef struct motorCtx_s {
    motorIF_t       *motorIF;
    feedback_t      *fb;
    bool            enabled;
    mtrDriveState_e mtrState;

    /* Control Loop */
    mtrDriveMode_e  ctrlMode;
    float           position; //forward the feedback data into here
    uint32_t        cmd;
    mtrDriveDir_e   dir;
    void (*ctrlLoop) (struct motorCtx_s *motor);
    void *ctrlParams;
} motorCtx_t;

typedef struct motorCtrlCtx_s{
    uint8_t     numMotors;
    motorCtx_t  mtrs[MAX_MOTORS];
    uint16_t    debugFlag;
    /* Add PID types here for each motor. Should it be in motorCtx?*/
} motorCtrlCtx_t;


resp_t motorInit(motorCtrlCtx_t *mtrCtrlCtx);
// Turn motor on/off
void setMotorEnable(motorCtx_t *motor, bool enable);
// Change motor control mode
void setDriveMode(motorCtx_t* motor, mtrDriveMode_e setMode);
void setDrivePwm(motorCtx_t *motor, int32_t setDrive);

// float percentOpen(float angle);
// void setTargetPercent(mtrState_t *mtrState, float target);

int motorCtrlInit(motorCtrlCtx_t *mtrCtrlCtx);


#endif