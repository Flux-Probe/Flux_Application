/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "fluxBleService.h"
#include "services/ans/ble_svc_ans.h"

/*** Maximum number of characteristics with the notify flag ***/
#define MAX_NOTIFY 5

/*Data buffers*/
float current_temp  = 25.0; // Celsius
float target_temp   = 33.0; // Celsius 
float motor_current = 0.0;  // Amps
int   motor_pos     = 0;    // Position/Angle in degrees 

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);

/*UUID's For services and characteristics*/
static const ble_uuid128_t flux_svr_svc_uuid = BLE_UUID128_INIT(
    0xA0,0x00,0x00,0x00,
    0xA1,0xA1,0xA1,0xA1,
    0xB1,0xB1,0xB1,0xB1,
    0xC1,0xC1,0xC1,0xC1
);
static const ble_uuid128_t flux_chr_cur_temp_uuid = BLE_UUID128_INIT(
    0xA1,0x00,0x00,0x00,
    0xA1,0xA1,0xA1,0xA1,
    0xB1,0xB1,0xB1,0xB1,
    0xC1,0xC1,0xC1,0xC1
);
static const ble_uuid128_t flux_chr_target_temp_uuid = BLE_UUID128_INIT(
    0xA2,0x00,0x00,0x00,
    0xA1,0xA1,0xA1,0xA1,
    0xB1,0xB1,0xB1,0xB1,
    0xC1,0xC1,0xC1,0xC1
);
static const ble_uuid128_t flux_chr_mtr_current_uuid = BLE_UUID128_INIT(
    0xA3,0x00,0x00,0x00,
    0xA1,0xA1,0xA1,0xA1,
    0xB1,0xB1,0xB1,0xB1,
    0xC1,0xC1,0xC1,0xC1
);
static const ble_uuid128_t flux_chr_mtr_pos_uuid = BLE_UUID128_INIT(
    0xA4,0x00,0x00,0x00,
    0xA1,0xA1,0xA1,0xA1,
    0xB1,0xB1,0xB1,0xB1,
    0xC1,0xC1,0xC1,0xC1
);

const struct ble_gatt_svc_def flux_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &flux_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &flux_chr_cur_temp_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &flux_chr_target_temp_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &flux_chr_mtr_current_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &flux_chr_mtr_pos_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {0}, // End of characteristics
        },
    },
    {0}, // End of services
};

static int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    MODLOG_DFLT(INFO, "Write Start\n");
    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        MODLOG_DFLT(INFO, "Write Len Err %d|%d\n", om_len, min_len, max_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {

        MODLOG_DFLT(INFO, "Error Writing to Value %d\n", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }
    else {
        MODLOG_DFLT(INFO, "Write successful\n");
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
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid;
    int rc;

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        MODLOG_DFLT(INFO, "Gatt Access Operation %d; conn_handle=%d attr_handle=%d\n",
                    ctxt->op, conn_handle, attr_handle);
    } else {
        MODLOG_DFLT(INFO, "Gatt Access Operation %d by NimBLE stack; attr_handle=%d\n",
                    ctxt->op, attr_handle);
    }

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        uuid = ctxt->chr->uuid;
        if (ble_uuid_cmp(uuid, &flux_chr_cur_temp_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om, &current_temp, sizeof(current_temp));
        }
        else if (ble_uuid_cmp(uuid, &flux_chr_target_temp_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om, &target_temp, sizeof(target_temp));
        }
        else if (ble_uuid_cmp(uuid, &flux_chr_mtr_current_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om, &motor_current, sizeof(motor_current));
        }
        else if (ble_uuid_cmp(uuid, &flux_chr_mtr_pos_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om, &motor_pos, sizeof(motor_pos));
        }
        else {
            goto unknown;
        }

        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        uuid = ctxt->chr->uuid;
        if (ble_uuid_cmp(uuid, &flux_chr_target_temp_uuid.u) == 0) {
            /* Sending Float value needs:
             * - 32 bits, LITTLE endian order
             * - 1 Bit = Sign (0 for positive)
             * - 8 Bits = Exponent
             * - 23 Bits = Mantissa
             * Byte Order: Mantissa - Exponent - Sign
             * EX:  Dec: 25.5
             *      Binary: 10011000000000000000000 - 10000011 - 0
             *      Hex: 00 00 CC 41
            */
            rc = gatt_svr_write(ctxt->om,
                                sizeof(target_temp),
                                sizeof(target_temp),
                                &target_temp, NULL);
            return rc;
        }
        else if (ble_uuid_cmp(uuid, &flux_chr_mtr_pos_uuid.u) == 0) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(motor_pos),
                                sizeof(motor_pos),
                                &motor_pos, NULL);
            return rc;
        }
        goto unknown;
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

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int gatt_svr_init(void)
{
    ble_svc_gap_init(); //Lets clients see device name, advertising info
    ble_svc_gatt_init(); //Basic GATT functionality. Lets clients see what services and characteristics are avaliable
    ble_gatts_count_cfg(flux_svr_svcs);
    ble_gatts_add_svcs(flux_svr_svcs);
    /*Used to advertise alert notifications to clients when things change*/
    // ble_svc_ans_init();

    return 0;
}
