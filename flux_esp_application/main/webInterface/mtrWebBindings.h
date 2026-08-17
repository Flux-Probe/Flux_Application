#ifndef _MTR_WEB_BINDINGS_H
#define _MTR_WEB_BINDINGS_H

#include "webApp.h"
#include "motorCtrl.h"

/*
 * Register motor telemetry and controls into the webapp.
 * Call once after startHttpServer(), before requests arrive.
 *
 * Per motor i:
 *   Monitors:  mtr<i>.pos, mtr<i>.enabled, mtr<i>.mode, mtr<i>.posSetpoint,
 *              mtr<i>.driveSetpoint, mtr<i>.dutyCmd, mtr<i>.state,
 *              mtr<i>.kp, mtr<i>.ki, mtr<i>.kd, mtr<i>.error,
 *              mtr<i>.integral, mtr<i>.prevError
 *
 *   Controls:  mtr<i>.enable, mtr<i>.mode, mtr<i>.pwmSetpoint,
 *              mtr<i>.posSetpoint, mtr<i>.gains ("kp,ki,kd")
 */
void motorCtrlRegisterWebBindings(webApp_t *web, motorCtrlCtx_t *mtrCtrl);

#endif