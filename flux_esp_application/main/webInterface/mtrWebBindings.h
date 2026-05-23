#ifndef _MTR_WEB_BINDINGS_H
#define _MTR_WEB_BINDINGS_H

#include "webApp.h"
#include "motorCtrl.h"

/*
 * Register motor telemetry and controls into the webapp.
 * Call once after startHttpServer(), before requests arrive.
 *
 * Per motor i:
 *   Monitors:  mtr<i>.pos, mtr<i>.enabled, mtr<i>.mode, mtr<i>.cmd, mtr<i>.dir,
 *              mtr<i>.state
 *
 *   Controls:  mtr<i>.enable, mtr<i>.mode, mtr<i>.speed
 */
void motorCtrlRegisterWebBindings(webApp_t *web, motorCtrlCtx_t *mtrCtrl);

#endif