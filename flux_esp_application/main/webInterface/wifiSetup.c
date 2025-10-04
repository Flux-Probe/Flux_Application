#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "wifiSetup.h"

// TODO: Place these into a config file to pull from.
// Add a way to choose and set the WIFI on the fly
#define WIFI_SSID "Krsikapa_Wifi"
#define WIFI_PASS "nicoSetThisUp23"
#define AP_SSID   "Flux-AP"
#define AP_PASS   "fluxpass"

// Needed for Logging module name
#define TAG "WIFI-MOD"

wifi_config_t dfltStaWifiCfg = {
    .sta = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    }
};
wifi_config_t dfltApWifiCfg = {
    .ap = {
        .ssid = AP_SSID,
        .ssid_len = strlen(AP_SSID),
        .password = AP_PASS,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA_WPA2_PSK
    }
};


static bool initialized = false;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        LOG_I("station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        LOG_I("station "MACSTR" leave, AID=%d, reason=%d", MAC2STR(event->mac),
                                                     event->aid, event->reason);
    }
}

resp_t wifi_conn_init(wifiConn_t *connection)
{
    wifi_init_config_t dfltWifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();

    if (initialized) {
        LOG_W("Wifi Module has already been initialized");
        return RESP_ERR;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    memcpy(&connection->initParams, &dfltWifiInitCfg, sizeof(wifi_init_config_t));

    ESP_ERROR_CHECK(esp_wifi_init(&connection->initParams));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        connection,
                                                        NULL));

    // Setup both AP and STA
    memcpy(&connection->staConfig, &dfltStaWifiCfg, sizeof(wifi_config_t));
    memcpy(&connection->apConfig, &dfltApWifiCfg, sizeof(wifi_config_t));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &connection->staConfig));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &connection->apConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set the following to true so this module doesnt get re-initialized
    initialized = true;
    LOG_I("Finished initializing WIFI Module");
    return RESP_OK;
}