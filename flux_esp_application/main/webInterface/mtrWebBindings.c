#include "mtrWebBindings.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * Getters — write a bare JSON value (no surrounding quotes for numbers)
 */
static void get_pos(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%.2f", ((motorCtx_t *)ctx)->position);
}

static void get_enabled(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%s", ((motorCtx_t *)ctx)->enabled ? "true" : "false");
}

static void get_mode(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%d", (int)((motorCtx_t *)ctx)->ctrlMode);
}

static void get_cmd(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%d", (int)((motorCtx_t *)ctx)->cmd);
}

static void get_dir(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%d", (int)((motorCtx_t *)ctx)->dir);
}

static void get_state(char *o, size_t n, void *ctx)
{
    snprintf(o, n, "%d", (int)((motorCtx_t *)ctx)->mtrState);
}

/**
 * Setters — parse the raw query-string value
 */
static void set_enable(const char *val, void *ctx)
{
    setMotorEnable((motorCtx_t *)ctx, strtol(val, NULL, 10) != 0);
}

static void set_mode(const char *val, void *ctx)
{
    setDriveMode((motorCtx_t *)ctx, (mtrDriveMode_e)strtol(val, NULL, 10));
}

static void set_pwm(const char *val, void *ctx)
{
    setDrivePwm((motorCtx_t *)ctx, (int32_t)strtol(val, NULL, 10));
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
        webAppRegisterMonitor(web, key, get_pos, m);

        snprintf(key, sizeof(key), "mtr%d.enabled", i);
        webAppRegisterMonitor(web, key, get_enabled, m);

        snprintf(key, sizeof(key), "mtr%d.mode", i);
        webAppRegisterMonitor(web, key, get_mode, m);

        snprintf(key, sizeof(key), "mtr%d.cmd", i);
        webAppRegisterMonitor(web, key, get_cmd, m);

        snprintf(key, sizeof(key), "mtr%d.dir", i);
        webAppRegisterMonitor(web, key, get_dir, m);

        snprintf(key, sizeof(key), "mtr%d.state", i);
        webAppRegisterMonitor(web, key, get_state, m);

        /*Setup the Setter controls */
        snprintf(key, sizeof(key), "mtr%d.enable", i);
        webAppRegisterControl(web, key, set_enable, m);

        snprintf(key, sizeof(key), "mtr%d.mode", i);
        webAppRegisterControl(web, key, set_mode, m);

        snprintf(key, sizeof(key), "mtr%d.speed", i);
        webAppRegisterControl(web, key, set_pwm, m);
    }
}
