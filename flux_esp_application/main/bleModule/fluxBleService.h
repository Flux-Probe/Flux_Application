/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef H_BLE_DYNAMIC_SERVICE_
#define H_BLE_DYNAMIC_SERVICE_

#include <stdbool.h>
#include "nimble/ble.h"
#include "host/ble_uuid.h"
#include "modlog/modlog.h"
#include "esp_peripheral.h"
#include "mainDefs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

typedef enum {
    COMMAND_CHR,
    RECIPE_WRITE_CHR,
    NOTIFICATION_CHR,
    MAX_CHR_CNT,
} charIdx_e;

typedef struct {
    uint16_t           *handle;
    uint16_t            conn_handle;
    bool                handle_init;
    bool                handle_sts;

    const ble_uuid128_t uuid;

} characteristics_t;

typedef struct {
    characteristics_t chars[MAX_CHR_CNT];
    ble_uuid128_t svcUuid;
} bleSvc_t;


/** GATT server. */
#define GATT_SVR_SVC_ALERT_UUID               0x1811
#define GATT_SVR_CHR_SUP_NEW_ALERT_CAT_UUID   0x2A47 //Not used
#define GATT_SVR_CHR_NEW_ALERT                0x2A46 //Not used
#define GATT_SVR_CHR_SUP_UNR_ALERT_CAT_UUID   0x2A48 //Not used
#define GATT_SVR_CHR_UNR_ALERT_STAT_UUID      0x2A45 //Not used
#define GATT_SVR_CHR_ALERT_NOT_CTRL_PT        0x2A44 //Not used

void gattSvrRegisterCb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gattSvrSubscribeCb(struct ble_gap_event *event);
void notifyCharUpdate(characteristics_t chr);
int  gatt_svr_init(bleSvc_t *bleSvc);
void start_ble_service(bleSvc_t *bleSvc);
int  bleNotify_send(const char *json);

#endif
