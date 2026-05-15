/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "fluxBleService.h"
#include "blePacketHandler.h"
#include "services/ans/ble_svc_ans.h"

/*** Maximum number of characteristics with the notify flag ***/
#define TAG "GATT_SVR"
#define NUM_SVCS 2
/* CHARS coming from application are in multiple packets. Use static buffers to
   read then process later
*/
#define RX_BUF_SIZE 4096
#define TX_BUF_SIZE 512

static uint8_t  rx_buf[RX_BUF_SIZE];
static uint16_t rx_len = 0;
static uint8_t  tx_buf[TX_BUF_SIZE];
static uint16_t tx_len = 0;

static int gattSvcAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

uint16_t handles[MAX_CHR_CNT];
bleSvc_t *fluxBleSvc;

const uint16_t flags[MAX_CHR_CNT] = {
    [COMMAND_CHR] = BLE_GATT_CHR_F_WRITE,
    [RECIPE_WRITE_CHR] = BLE_GATT_CHR_F_WRITE,
    [NOTIFICATION_CHR] = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
};

bleSvc_t dfltFluxBleSvc = {
    .svcUuid = BLE_UUID128_INIT(
        0xFB,0x34,0x9B,0x5f,
        0x80,0x00,0x00,0x80,
        0x00,0x10,0x00,0x00,
        0xE0,0xFF,0x00,0x00,
    ),
    .chars = {
        [COMMAND_CHR] = {
            .uuid = BLE_UUID128_INIT(
                0xFB,0x34,0x9B,0x5f,
                0x80,0x00,0x00,0x80,
                0x00,0x10,0x00,0x00,
                0xE1,0xFF,0x00,0x00,
            ),
        },
        [RECIPE_WRITE_CHR] = {
            .uuid = BLE_UUID128_INIT(
                0xFB,0x34,0x9B,0x5f,
                0x80,0x00,0x00,0x80,
                0x00,0x10,0x00,0x00,
                0xE2,0xFF,0x00,0x00,
            ),
        },
        [NOTIFICATION_CHR] = {
            .uuid = BLE_UUID128_INIT(
                0xFB,0x34,0x9B,0x5f,
                0x80,0x00,0x00,0x80,
                0x00,0x10,0x00,0x00,
                0xE3,0xFF,0x00,0x00,
            ),
        }
    }
};

/*
    NOTE: Seems like this needs to be global. When this was put in the
    configGatts function, the esp kept crashing...
*/
struct ble_gatt_svc_def flux_svr_svcs[NUM_SVCS] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &dfltFluxBleSvc.svcUuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &dfltFluxBleSvc.chars[COMMAND_CHR].uuid.u,
                .access_cb = gattSvcAccess,
                .val_handle = &handles[COMMAND_CHR],
                .flags = flags[COMMAND_CHR],
            },
            {
                .uuid = &dfltFluxBleSvc.chars[RECIPE_WRITE_CHR].uuid.u,
                .access_cb = gattSvcAccess,
                .val_handle = &handles[RECIPE_WRITE_CHR],
                .flags = flags[RECIPE_WRITE_CHR],
            },
            {
                .uuid = &dfltFluxBleSvc.chars[NOTIFICATION_CHR].uuid.u,
                .access_cb = gattSvcAccess,
                .val_handle = &handles[NOTIFICATION_CHR],
                .flags = flags[NOTIFICATION_CHR],
            },
            {0}, // End of characteristics
        },
    },
    {0}, // End of services
};

static int gattSvrWrite(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                        void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    LOG_V("Write Start\n");
    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        LOG_E("Write Len Err %d|%d\n", om_len, min_len, max_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        LOG_E("Error Writing to Value %d\n", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }
    else {
        LOG_V("Write successful\n");
    }

    return 0;
}

/**
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/
static int gattSvcAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (!fluxBleSvc) {
        goto unknown;
    }
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        LOG_V("Gatt Access Operation %d; conn_handle=%d attr_handle=%d\n",
              ctxt->op, conn_handle, attr_handle);
    } else {
        LOG_V("Gatt Access Operation %d by NimBLE stack; attr_handle=%d\n",
              ctxt->op, attr_handle);
    }

    /*
        Confirm what characteristic is being targetted. Store the IDX.
        return err if it is not valid
    */
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    int chrIdx;
    for (chrIdx = COMMAND_CHR; chrIdx < MAX_CHR_CNT; chrIdx++) {
        if (ble_uuid_cmp(uuid, &fluxBleSvc->chars[chrIdx].uuid.u) == 0) {
            break;
        }
    }

    if (chrIdx == MAX_CHR_CNT) {
        LOG_W("character being access has an unknown uuid");
        goto unknown;
    }

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        rc = os_mbuf_append(ctxt->om, tx_buf, tx_len);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        rc = gattSvrWrite(ctxt->om, 1, RX_BUF_SIZE, rx_buf, &rx_len);
        if (rc == 0) {
            LOG_I("Len: %d", rx_len);
            blePacketHandler_onWrite(chrIdx, rx_buf, rx_len);
        }
        return rc;
    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void gattSvrRegisterCb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        LOG_V("registered service %s with handle=%d\n",
              ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        LOG_V("registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        LOG_V("registering descriptor %s with handle=%d\n",
              ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

void gattSvrSubscribeCb(struct ble_gap_event *event)
{
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                event->subscribe.attr_handle);
    }

    int chrIdx;
    for (chrIdx = COMMAND_CHR; chrIdx < MAX_CHR_CNT; chrIdx++) {
        if (event->subscribe.attr_handle == *fluxBleSvc->chars[chrIdx].handle) {
            break;
        }
    }
    if (chrIdx == MAX_CHR_CNT) {
        LOG_E("Subscribe CB failed as handle was not found");
        return;
    }

    fluxBleSvc->chars[chrIdx].conn_handle = event->subscribe.conn_handle;
    fluxBleSvc->chars[chrIdx].handle_init = true;
    fluxBleSvc->chars[chrIdx].handle_sts = event->subscribe.cur_notify;

    LOG_V("subscribeCb: %d|%d|%d", chrIdx,
          fluxBleSvc->chars[chrIdx].handle_init,
          fluxBleSvc->chars[chrIdx].handle_sts);
}

/* When sending a notification, Call bleNotify_send with the data in JSON format
   to send. This will copy into a buffer and call ble_gatts_notify with the appropriate
   handle. This then calls gattSvcAccess as a read event which then sends out the data
   within tx_buf.
   */
void notifyCharUpdate(characteristics_t chr)
{
    int rc =  0;
    if (chr.handle_init && chr.handle_sts) {
        rc = ble_gatts_notify(chr.conn_handle, *chr.handle);
        if (rc != 0) {
            LOG_I("Notify Err: %d", rc);
        }
    }
}

int bleNotify_send(const char *json)
{
    if (!fluxBleSvc) return -1;
    int jlen = (int)strlen(json);
    if (jlen >= TX_BUF_SIZE) {
        LOG_E("Notification too large: %d bytes", jlen);
        return -1;
    }
    memcpy(tx_buf, json, jlen);
    tx_len = (uint16_t)jlen;
    notifyCharUpdate(fluxBleSvc->chars[NOTIFICATION_CHR]);
    return 0;
}

/*
    Populate the svc pointer with the defaults and update the local pointer to
    be used in the access_cb function
*/
void loadDefaults(bleSvc_t *bleSvc)
{
    memcpy(bleSvc, &dfltFluxBleSvc, sizeof(bleSvc_t));
    bleSvc->chars[COMMAND_CHR].handle      = &handles[COMMAND_CHR];
    bleSvc->chars[RECIPE_WRITE_CHR].handle = &handles[RECIPE_WRITE_CHR];
    bleSvc->chars[NOTIFICATION_CHR].handle = &handles[NOTIFICATION_CHR];
    fluxBleSvc = bleSvc;
}

int configGatts(bleSvc_t *bleSvc)
{
    loadDefaults(bleSvc);

    int resp = ble_gatts_count_cfg(flux_svr_svcs);
    if (resp != 0) {
        LOG_E("Error updating GATT service counter");
        return resp;
    }
    resp = ble_gatts_add_svcs(flux_svr_svcs);
    if (resp != 0) {
        LOG_E("Error updating GATT service counter");
        return resp;
    }

    return resp;
}

int gatt_svr_init(bleSvc_t *bleSvc)
{
    /*
        Basic GATT functionality. Lets clients see what services
        and characteristics are avaliable
    */
    ble_svc_gatt_init();
    esp_log_level_set(TAG, ESP_LOG_ERROR | ESP_LOG_DEBUG); // Setting debug

    int resp = configGatts(bleSvc);

    /*Used to advertise alert notifications to clients when things change*/
    // ble_svc_ans_init();

    return resp;
}
