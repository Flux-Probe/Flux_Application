
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
    MTR_COAST,
} mtrDriveDir_e;
typedef struct motorIF_s {
    // resp_t (*initDriver)  (struct motorIF_s *mtrDriver);
    resp_t (*enable)      (struct motorIF_s *mtrDriver);
    resp_t (*disable)     (struct motorIF_s *mtrDriver);
    resp_t (*getDrive)    (struct motorIF_s *mtrDriver);
    // resp_t (*getCurrent)  (struct motorIF_s *mtrDriver);
    resp_t (*getStatus)   (struct motorIF_s *mtrDriver);
    resp_t (*setDrive)    (struct motorIF_s *mtrDriver, uint32_t targetPwm);
    resp_t (*setDir)      (struct motorIF_s *mtrDriver, mtrDriveDir_e dir);
    resp_t (*resetMotor)  (struct motorIF_s *mtrDriver);

    void *privCtx;
} motorIF_t;

#endif