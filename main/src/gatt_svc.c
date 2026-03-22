/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "gatt_svc.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "host/ble_uuid.h"
#include <stdint.h>

static int atmospheric_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

static const ble_uuid16_t atmospheric_svc_uuid = BLE_UUID16_INIT(0x181A);

struct atm_data {
    int16_t temp;
    uint16_t press;
    uint16_t hum;
} __attribute__((packed));

static struct atm_data atmospheric_chr_val = {0,0,0};
static uint16_t atmospheric_chr_val_handle;
static const ble_uuid128_t atmospheric_chr_uuid = {
    .u.type = BLE_UUID_TYPE_128,
    .value = {
        0xb2, 0xb5, 0x48, 0x3e, 0x36, 0xe1, 0x46, 0x77, 
        0xb8, 0xf5, 0xea, 0x17, 0x36, 0x1b, 0x26, 0xa8
    }
};

static uint16_t atmospheric_chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool atmospheric_chr_conn_handle_inited = false;
static bool atmospheric_ind_status = false;

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &atmospheric_svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {              
              .uuid = (ble_uuid_t *)&atmospheric_chr_uuid,
              .access_cb = atmospheric_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
              .val_handle = &atmospheric_chr_val_handle},
             {
                 0, /* No more characteristics in this service. */
             }}},
    {
        0, /* No more services. */
    },
};

static int atmospheric_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Local variables */
    int rc = 0;

    /* Handle access events */
    switch (ctxt->op) {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d",
                     attr_handle);
        }

        /* Verify attribute handle */
        if (attr_handle == atmospheric_chr_val_handle) {
            /* Update access buffer value */
            rc = os_mbuf_append(ctxt->om, &atmospheric_chr_val,
                                sizeof(atmospheric_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto error;

    /* Unknown event */
    default:
        goto error;
    }

error:
    ESP_LOGE(
        TAG,
        "unexpected access operation to atmospheric characteristic, opcode: %d",
        ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

/* Public functions */
void send_atmospheric_indication(void) {
    if (atmospheric_ind_status && atmospheric_chr_conn_handle_inited) {
        ble_gatts_indicate(atmospheric_chr_conn_handle,
                           atmospheric_chr_val_handle);
        ESP_LOGI(TAG, "atmospheric data indication sent!");
    }
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *      1. Update subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    /* Check attribute handle */
    if (event->subscribe.attr_handle == atmospheric_chr_val_handle) {
        /* Update subscription status */
        atmospheric_chr_conn_handle = event->subscribe.conn_handle;
        atmospheric_chr_conn_handle_inited = true;
        atmospheric_ind_status = event->subscribe.cur_indicate;
    }
}

void gatt_svr_reset_atmospheric_subscription(void) {
    atmospheric_chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    atmospheric_chr_conn_handle_inited = false;
    atmospheric_ind_status = false;
}

void set_atm_values(uint16_t temp, int16_t press, int16_t hum) {
   atmospheric_chr_val.temp = temp; 
   atmospheric_chr_val.press = press; 
   atmospheric_chr_val.hum = hum; 
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc = 0;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
