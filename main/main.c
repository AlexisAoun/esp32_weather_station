#include "common.h"
#include "gap.h"
#include "gatt_svc.h"

/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

static void atm_sensor_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "Atm sensor task has been started!");

    /* Loop forever */
    while (1) {
        /* Send heart rate indication if enabled */
        send_atmospheric_indication();

        /* Sleep */
        vTaskDelay(1000);
    }

    /* Clean up at exit */
    vTaskDelete(NULL);
}

void app_main(void) {
    /* Local variables */
    BaseType_t rc = 0;
    esp_err_t ret;

    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations
     */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }
#endif

    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    rc = xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL,
                                5, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "failed to create NimBLE host task");
        return;
    }

    rc = xTaskCreate(atm_sensor_task, "ATM sensor", 4 * 1024, NULL, 5, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "failed to create heart rate task");
        return;
    }
    return;
}
// void app_main(void)
// {
//     sensor_data_t sensor_data; 
//     sensors_config_t sensors_config;
//
//     ESP_ERROR_CHECK(init_sensors(&sensors_config));
//
//     while(true) {
//         ESP_ERROR_CHECK(read_sensors_data(sensors_config, &sensor_data));
//         printf("Temp: %.2f°C, Hum : %.2f%%, Pressure: %.2f Pa\n", sensor_data.temperature, sensor_data.humidity, sensor_data.pressure);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//    }
// }
