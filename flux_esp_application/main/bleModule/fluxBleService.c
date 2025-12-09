/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "fluxBleService.h"

#define TAG "BLE_SVR"
#define BLE_service_ADD 1
#define BLE_service_DELETE 0

static int gap_handler(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
static const char *dev_name = "FLUX_BLE_HOST";

/**
 * Logs information about a connection to the console.
 */
static void print_conn_desc(struct ble_gap_conn_desc *desc)
{
    /* TODO: Make the print_addr behind a log level*/
    LOG_W("handle=%d our_ota_addr_type=%d our_ota_addr=",
          desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    LOG_W(" our_id_addr_type=%d our_id_addr=", desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    LOG_W(" peer_ota_addr_type=%d peer_ota_addr=", desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    LOG_W(" peer_id_addr_type=%d peer_id_addr=", desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    LOG_W(" conn_itvl=%d conn_latency=%d supervision_timeout=%d encrypted=%d "
          "authenticated=%d bonded=%d\n", desc->conn_itvl, desc->conn_latency,
          desc->supervision_timeout, desc->sec_state.encrypted,
          desc->sec_state.authenticated, desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        LOG_E("error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /*
        RFI: Could possibly add a response set of fields below similar to above.
        Not sure if needed yet...
    */

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_handler, NULL);
    if (rc != 0) {
        LOG_E("error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * service uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                              service.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int gap_handler(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        LOG_V("connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
        }
        else {
            /* Connection failed; resume advertising. */
            start_advertising();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        LOG_V("disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);

        /* Connection terminated; resume advertising. */
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        LOG_V("connection updated; status=%d ", event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        LOG_V("\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        LOG_V("advertise complete; reason=%d", event->adv_complete.reason);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        LOG_V("notify_tx; conn_handle=%d attr_handle=%d sts=%d indication=%d",
              event->notify_tx.conn_handle, event->notify_tx.attr_handle,
              event->notify_tx.status, event->notify_tx.indication);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        LOG_V("subscribe; conn_handle=%d attr_handle=%d reason=%d prevn=%d "
              "curn=%d previ=%d curi=%d\n", event->subscribe.conn_handle,
              event->subscribe.attr_handle, event->subscribe.reason,
              event->subscribe.prev_notify, event->subscribe.cur_notify,
              event->subscribe.prev_indicate, event->subscribe.cur_indicate);
        gattSvrSubscribeCb(event);
        return 0;

    case BLE_GAP_EVENT_MTU:
        LOG_V("mtu update; conn_handle=%d cid=%d mtu=%d\n",
              event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
        return 0;
    }
    return 0;
}

static void service_on_reset(int reason)
{
    LOG_E("Resetting state; reason=%d\n", reason);
}

static void service_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        LOG_E("error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    LOG_V("Device Address: ");
    print_addr(addr_val);
    LOG_V("\n");
    /* Begin advertising. */
    start_advertising();
}

static void service_host_task(void *param)
{
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void start_ble_service(bleSvc_t *bleSvc)
{
    int rc;

    esp_log_level_set(TAG, ESP_LOG_ERROR | ESP_LOG_DEBUG); // Setting debug
    esp_log_level_set("NimBLE", ESP_LOG_ERROR); // Setting debug

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        LOG_E("Failed to init nimble %d ", ret);
        return;
    }
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = service_on_reset;
    ble_hs_cfg.sync_cb = service_on_sync;
    ble_hs_cfg.gatts_register_cb = gattSvrRegisterCb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init(); //Lets clients see device name, advertising info
    rc = gatt_svr_init(bleSvc);
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set(dev_name);
    assert(rc == 0);

    nimble_port_freertos_init(service_host_task);
}
