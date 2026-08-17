
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

typedef struct {
    int64_t t_prev;
    int64_t t_start;
    int64_t t_lastDirChanged;
} positionControlTimeParams_t;

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float error;
    float prevError;

    float minOut;
    float maxOut;
} pidLoop_t;


typedef struct motorCtx_s {
    uint8_t         idx;
    bool            enabled;
    mtrDriveState_e mtrState;

    /* Control Loop */
    mtrDriveMode_e  ctrlMode;
    float           position; //forward the feedback data into here
    struct {
        float upper;
        float lower;
    } limits; //forward the feedback data into here
    float           posSetpoint;
    float           driveSetpoint;
    float           driveCmd;

    motorIF_t       *motorIF;
    feedback_t      *fb;
    void (*ctrlLoop) (struct motorCtx_s *motor);
    pidLoop_t       pid;
} motorCtx_t;

typedef struct motorCtrlCtx_s{
    uint8_t     numMotors;
    motorCtx_t  mtrs[MAX_MOTORS];
    uint16_t    debugFlag;
    /* Add PID types here for each motor. Should it be in motorCtx?*/
} motorCtrlCtx_t;


// Turn motor on/off
void setMotorEnable(uint8_t idx, bool enable);
// Change motor control mode
void setDriveMode(uint8_t idx, mtrDriveMode_e setMode);
void setTargetPwm(uint8_t idx, float setDrive);
void setTargetPos(uint8_t idx, float setDrive);
// Update PID loop gains for a motor; blocks until motorControlTask is between cycles
resp_t setLoopGains(uint8_t idx, pidLoop_t gains);

resp_t motorCtrlInit(motorCtrlCtx_t *mtrCtrlCtx);


#endif