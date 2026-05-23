#ifndef _WEBAPP_H
#define _WEBAPP_H

#include "esp_http_server.h"
#include "mainDefs.h"

#define WEBAPP_MAX_MONITORS 32
#define WEBAPP_MAX_CONTROLS 16
#define WEBAPP_KEY_LEN      32

/*
 * getter: writes a JSON value (unquoted) into out — e.g. "123.4", "true", "0"
 * setter: receives the raw query-string value to parse — e.g. "200", "1"
 */
typedef void (*webAppGetter_t)(char *out, size_t outLen, void *ctx);
typedef void (*webAppSetter_t)(const char *val, void *ctx);

/**
 * Monitor and Control structs are how the system can dynamically setup fields to
 * track and execute commands through the web application.
 *
 * Currently monitor is used as a "Get All" fields listed, but in the future can
 * be broken up into multiple arrays "motorCtrlMonitor", "tempProbeMonitor",
 * "statusMonitor" for other logging
 *
 */
typedef struct {
    char            key[WEBAPP_KEY_LEN];
    webAppGetter_t  get;
    void           *ctx;
} webAppMonitor_t;

typedef struct {
    char            key[WEBAPP_KEY_LEN];
    webAppSetter_t  set;
    void           *ctx;
} webAppControl_t;

typedef struct {
    httpd_config_t  config;
    httpd_handle_t  server;

    webAppMonitor_t monitors[WEBAPP_MAX_MONITORS];
    int             monitorCount;

    webAppControl_t controls[WEBAPP_MAX_CONTROLS];
    int             controlCount;
} webApp_t;

resp_t webAppRegisterMonitor(webApp_t *web, const char *key, webAppGetter_t get, void *ctx);
resp_t webAppRegisterControl(webApp_t *web, const char *key, webAppSetter_t set, void *ctx);

resp_t startHttpServer(webApp_t *web);

#endif
