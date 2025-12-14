#ifndef _WEBAPP
#define _WEBAPP

#include "esp_http_server.h"
#include "motorCtrl.h"

typedef enum {
    URI_HOME_GET,
    URI_HOME_SET,
    URI_SET_PCT_GET,
} uriMethodType_e;

typedef struct {
    httpd_config_t config;
    httpd_handle_t server;
} webapp_t;

void start_http_server(webapp_t *web, mtrState_t *mtr);


#endif