
#ifndef _MOTORDRIVER_IF
#define _MOTORDRIVER_IF

#include <stdbool.h>
#include <stdint.h>
#include "mainDefs.h"

// Mtr driving Macros
#define HIGH 1
#define LOW  0

typedef struct {
    uint32_t pinHigh;   // Set pin High when moving forward, Low when reverse
    uint32_t pinLow;    // Set pin Low when moving forward, High when reverse
} mtrCfg_t;

typedef enum {
    MTR_OK,
    MTR_ERROR,
} mtrDriverStatus_e;

typedef enum {
    MTR_STOP,
    MTR_FORWARD,
    MTR_REVERSE,
} mtrDriveDir_e;

typedef struct {
    mtrDriverStatus_e   status;
    bool                enabled;
    uint16_t            debugFlag;
    mtrCfg_t           *cfg;

    /*
        TODO: Use eventually to keep track of the running state of
        the pins level. Use gpio_get_level().
        For now, just update after we set the pins as long as they are
        successful
    */
    int currPinHigh;
    int currPinLow;

    // Replace this with the specific encoder driver used
    // as5600_cfg_t encoderCfg;
} mtrState_t;

typedef struct motorIF_s {
    // resp_t (*initDriver)  (struct motorIF_s *mtrDriver);
    resp_t (*enable)      (struct motorIF_s *mtrDriver);
    resp_t (*disable)     (struct motorIF_s *mtrDriver);
    resp_t (*getDrive)    (struct motorIF_s *mtrDriver);
    // resp_t (*getCurrent)  (struct motorIF_s *mtrDriver);
    resp_t (*getStatus)   (struct motorIF_s *mtrDriver);
    resp_t (*setDrive)    (struct motorIF_s *mtrDriver, float targetPwm);
    resp_t (*resetMotor)  (struct motorIF_s *mtrDriver);

    void *privCtx;
} motorIF_t;

#endif