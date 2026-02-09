#ifndef SENSORS_H
#define SENSORS_H

#include "esp_err.h"
#include "esp_newlib.h"
#include <driver/i2c_master.h>

// Looking at the docs 0x1700 until 0x2000 is not used by any build in errors
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
#define BASE_ERROR_CODE 0x1700
#define ESP_ERR_BMP280_INIT_FAIL BASE_ERROR_CODE
#define ESP_ERR_AHT20_TIMEOUT (BASE_ERROR_CODE + 1)
#define ESP_ERR_AHT20_CORRUPT_DATA (BASE_ERROR_CODE + 2)

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} bmp280_calibration_t;

typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t aht20_handle;
    i2c_master_dev_handle_t bmp280_handle;
    bmp280_calibration_t bmp280_calibration; 

} sensors_config_t;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
} sensor_data_t;

esp_err_t init_sensors(sensors_config_t *sensors_config);
esp_err_t read_sensors_data(sensors_config_t sensors_config, sensor_data_t *data);

#endif
