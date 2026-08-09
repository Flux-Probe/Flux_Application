#include "mtrWebBindings.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * Getters — write a bare JSON value (no surrounding quotes for numbers)
 */
static void getPos(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%.2f", ((motorCtx_t *)ctx)->position);
}

static void getEnabled(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%s", ((motorCtx_t *)ctx)->enabled ? "true" : "false");
}

static void getMode(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%d", (int)((motorCtx_t *)ctx)->ctrlMode);
}

static void getPosSetpoint(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%.2f", (float)((motorCtx_t *)ctx)->posSetpoint);
}

static void getDriveSetpoint(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%.2f", (float)((motorCtx_t *)ctx)->driveSetpoint);
}

static void getDutyCmd(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%.2f", (float)((motorCtx_t *)ctx)->driveCmd);
}

static void getState(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%d", (int)((motorCtx_t *)ctx)->mtrState);
}

/**
 * Setters — parse the raw query-string value
 */
static void setEnable(const char *val, void *ctx)
{
    setMotorEnable(*(uint8_t *)ctx, strtol(val, NULL, 10) != 0);
}

static void setMode(const char *val, void *ctx)
{
    setDriveMode(*(uint8_t *)ctx, (mtrDriveMode_e)strtol(val, NULL, 10));
}

static void setPwm(const char *val, void *ctx)
{
    setTargetPwm(*(uint8_t *)ctx, (float)strtof(val, NULL));
}

static void setPos(const char *val, void *ctx)
{
    setTargetPos(*(uint8_t *)ctx, (float)strtof(val, NULL));
}

/**
 * Registration
 */
void motorCtrlRegisterWebBindings(webApp_t *web, motorCtrlCtx_t *mtrCtrl)
{
    char key[WEBAPP_KEY_LEN];

    for (int i = 0; i < mtrCtrl->numMotors; i++) {
        motorCtx_t *m = &mtrCtrl->mtrs[i];

        /* Setup the Getter controls*/
        snprintf(key, sizeof(key), "mtr%d.pos", i);
        webAppRegisterMonitor(web, key, getPos, m);

        snprintf(key, sizeof(key), "mtr%d.enabled", i);
        webAppRegisterMonitor(web, key, getEnabled, m);

        snprintf(key, sizeof(key), "mtr%d.mode", i);
        webAppRegisterMonitor(web, key, getMode, m);

        snprintf(key, sizeof(key), "mtr%d.posSetpoint", i);
        webAppRegisterMonitor(web, key, getPosSetpoint, m);

        snprintf(key, sizeof(key), "mtr%d.driveSetpoint", i);
        webAppRegisterMonitor(web, key, getDriveSetpoint, m);

        snprintf(key, sizeof(key), "mtr%d.dutyCmd", i);
        webAppRegisterMonitor(web, key, getDutyCmd, m);

        snprintf(key, sizeof(key), "mtr%d.state", i);
        webAppRegisterMonitor(web, key, getState, m);

        /*Setup the Setter controls */
        snprintf(key, sizeof(key), "mtr%d.enable", i);
        webAppRegisterControl(web, key, setEnable, &m->idx);

        snprintf(key, sizeof(key), "mtr%d.mode", i);
        webAppRegisterControl(web, key, setMode, &m->idx);

        snprintf(key, sizeof(key), "mtr%d.pwmSetpoint", i);
        webAppRegisterControl(web, key, setPwm, &m->idx);

        snprintf(key, sizeof(key), "mtr%d.posSetpoint", i);
        webAppRegisterControl(web, key, setPos, &m->idx);
    }
}
