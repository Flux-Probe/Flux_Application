#ifndef _WIFISETUP
#define _WIFISETUP

#include "mainDefs.h"
#include "esp_wifi.h"

typedef struct {
    wifi_init_config_t initParams;
    wifi_config_t staConfig;
    wifi_config_t apConfig;
    // TODO: Prolly add state to all modules
} wifiConn_t;

resp_t wifi_conn_init(wifiConn_t *connection);

#endif