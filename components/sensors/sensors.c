/*  Datasheets : 
 *  BMP280 : https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 *  AHT20 : https://asairsensors.com/wp-content/uploads/2021/09/Data-Sheet-AHT20-Humidity-and-Temperature-Sensor-ASAIR-V1.0.03.pdf
 */

#include <stdint.h>
#include <stdio.h>
#include <driver/i2c_master.h>
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "include/sensors.h"
#include "sensors.h"

// I2C bus 
#define I2C_PORT 0
#define SCL_IO_NUM 22
#define SDA_IO_NUM 21
#define GLITCH_IGNORE_CNT 7
#define ENABLE_INTERNAL_PULLUP true

//AHT20
#define AHT20_DEVICE_ADDRESS 0x38
#define AHT20_SCL_SPEEH_HZ 100000
#define AHT20_MEASUREMENT_COMMAND {0xAC, 0x33, 0x00}
#define AHT20_MEASUREMENT_SIZE 7

//BMP280
#define BMP280_DEVICE_ADDRESS 0x77
#define BMP280_SCL_SPEEH_HZ 100000
#define BMP280_ID_REGISTER_ADDRESS 0xD0
#define BMP280_EXPECTED_ID 0x58
#define BMP280_CALIB_ADDRESS 0x88
#define BMP280_PRESSION_ADDRESS 0xF7
#define BMP280_MEASUREMENT_SIZE 6
#define BMP280_CONFIG_ADDRESS 0xF4
#define BMP280_OVERSAMPLING_CONFIG 0x93
#define BMP280_CALIBRATION_DATA_SIZE 24

typedef struct {
    float temp;
    float hum;
} aht20_measurement_t;

typedef struct {
    float temp;
    float press;
} bmp280_measurement_t;


static esp_err_t init_i2c_port(i2c_master_bus_handle_t* bus_handle) {
	i2c_master_bus_config_t i2c_mst_config = {
	    .clk_source = I2C_CLK_SRC_DEFAULT,
	    .i2c_port = I2C_PORT,
	    .scl_io_num = SCL_IO_NUM,
	    .sda_io_num = SDA_IO_NUM,
	    .glitch_ignore_cnt = GLITCH_IGNORE_CNT,
	    .flags.enable_internal_pullup = ENABLE_INTERNAL_PULLUP,
	};

	esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, bus_handle);
    return ret;
}

static esp_err_t init_bm280(i2c_master_dev_handle_t* dev_handle, i2c_master_bus_handle_t bus_handle, bmp280_calibration_t* calib) {
	i2c_device_config_t dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = BMP280_DEVICE_ADDRESS,
	    .scl_speed_hz = BMP280_SCL_SPEEH_HZ,
	};

	esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }

	vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t id[1];
    uint8_t id_reg = BMP280_ID_REGISTER_ADDRESS;
    esp_err_t ret2 = i2c_master_transmit_receive(*dev_handle, &id_reg, 1, id, 1, -1);
    if (ret2 != ESP_OK) {
        return ret2;
    }

    // For some reason can't get ESP_RETURN_ON_FALSE working, doing it manually
    if (id[0] != BMP280_EXPECTED_ID) {
        return ESP_ERR_BMP280_INIT_FAIL;
    }

    // configuration of the oversampling
    uint8_t config[] = {BMP280_CONFIG_ADDRESS, BMP280_OVERSAMPLING_CONFIG};
    esp_err_t ret3 = i2c_master_transmit(*dev_handle, config, 2, -1);
    if (ret3 != ESP_OK) {
        return ret3;
    }

	vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t calib_reg = BMP280_CALIB_ADDRESS;
    uint8_t calib_data[BMP280_CALIBRATION_DATA_SIZE];
    esp_err_t ret4 = i2c_master_transmit_receive(*dev_handle, &calib_reg, 1, calib_data, BMP280_CALIBRATION_DATA_SIZE, -1);
    if (ret4 != ESP_OK) {
        return ret4;
    }

    calib->dig_t1 = (uint16_t)calib_data[1] << 8 | calib_data[0];
    calib->dig_t2 = (int16_t)calib_data[3] << 8 | calib_data[2];
    calib->dig_t3 = (int16_t)calib_data[5] << 8 | calib_data[4];
    calib->dig_p1 = (uint16_t)calib_data[7] << 8 | calib_data[6];
    calib->dig_p2 = (int16_t)calib_data[9] << 8 | calib_data[8];
    calib->dig_p3 = (int16_t)calib_data[11] << 8 | calib_data[10];
    calib->dig_p4 = (int16_t)calib_data[13] << 8 | calib_data[12];
    calib->dig_p5 = (int16_t)calib_data[15] << 8 | calib_data[14];
    calib->dig_p6 = (int16_t)calib_data[17] << 8 | calib_data[16];
    calib->dig_p7 = (int16_t)calib_data[19] << 8 | calib_data[18];
    calib->dig_p8 = (int16_t)calib_data[21] << 8 | calib_data[20];
    calib->dig_p9 = (int16_t)calib_data[23] << 8 | calib_data[22];

    return ESP_OK;
}

static esp_err_t bmp280_read(i2c_master_dev_handle_t dev_handle, uint8_t* raw_data) {
    uint8_t press_address[1] = {BMP280_PRESSION_ADDRESS};
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, press_address, 1, raw_data, BMP280_MEASUREMENT_SIZE, -1);
    return ret;
}

static float bmp280_read_temp(uint8_t* raw_data, bmp280_calibration_t calib, uint32_t* fine_temp) {
    int32_t adc_temp = (int32_t)raw_data[3] << 12 | raw_data[4] << 4 | raw_data[5] >> 4;
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)calib.dig_t1 << 1))) * (int32_t)calib.dig_t2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)calib.dig_t1) * ((adc_temp >> 4) - (int32_t)calib.dig_t1) >> 12) * (int32_t)calib.dig_t3) >> 14);

    *fine_temp = var1 + var2;
    int32_t T = (*fine_temp * 5 + 128) >> 8;
    return (float)T / 100;
}

// calculations as per the BMP280 datasheet
static uint32_t bmp280_read_press(uint8_t* raw_data, bmp280_calibration_t calib, uint32_t fine_temp) {
    int64_t var1, var2, p;
    int32_t adc_press = (int32_t)raw_data[0] << 12 | raw_data[1] << 4 | raw_data[2] >> 4;

    var1 = ((int64_t)fine_temp) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_p6;
    var2 = var2 + ((var1*(int64_t)calib.dig_p5)<<17);
    var2 = var2 + (((int64_t)calib.dig_p4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_p3)>>8) + ((var1 * (int64_t)calib.dig_p2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib.dig_p1)>>33;
    if (var1 == 0)
    {
    return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_press;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)calib.dig_p9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib.dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_p7)<<4);
    return (uint32_t)p;
}

static bmp280_measurement_t compute_bmp280_raw_data(uint8_t* raw_data, bmp280_calibration_t calib) {
    bmp280_measurement_t output;
    uint32_t fine_temp;

    output.temp = bmp280_read_temp(raw_data, calib, &fine_temp);
    output.press = (float)bmp280_read_press(raw_data, calib, fine_temp) / 256;

    return output;
}

static esp_err_t init_aht20(i2c_master_dev_handle_t* dev_handle, i2c_master_bus_handle_t bus_handle) {
	i2c_device_config_t dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = AHT20_DEVICE_ADDRESS,
	    .scl_speed_hz = AHT20_SCL_SPEEH_HZ,
	};

	esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    return ret;
}

static uint8_t aht20_calc_crc(uint8_t *data, uint8_t len) 
{
	uint8_t i;
	uint8_t byte;
	uint8_t crc = 0xFF;
	
	for (byte = 0; byte < len; byte++) {
	    crc ^= data[byte];
	    for (i = 8; i > 0; --i) {
	        if ((crc & 0x80) != 0) {
	            crc = (crc << 1) ^ 0x31;
	        } else {
	            crc = crc << 1;
	        }
	    }
	}
	
	return crc;
}

static esp_err_t aht20_read(i2c_master_dev_handle_t dev_handle, uint8_t* data) {
	uint8_t command[3] = AHT20_MEASUREMENT_COMMAND;
	esp_err_t ret = i2c_master_transmit(dev_handle, command, 3, -1);
    if (ret != ESP_OK) {
        return ret; 
    }
	
	uint8_t status; 
	int timeout = 10;

    // AHT20 is ready for measurement when bit[7] of the status word is 0
    do {
	    vTaskDelay(pdMS_TO_TICKS(80)); 
	    i2c_master_receive(dev_handle, &status, 1, -1);
	    timeout--;
	} while ((status & 1) && timeout > 0);
	
	if (timeout == 0) {
        return ESP_ERR_AHT20_TIMEOUT;
	} 

	esp_err_t ret2 = i2c_master_receive(dev_handle, data, AHT20_MEASUREMENT_SIZE, -1);
    if (ret2 != ESP_OK) {
        return ret2; 
    }

    //CRC is the last byte
	uint8_t crc = aht20_calc_crc(data, AHT20_MEASUREMENT_SIZE-1);

    if (crc != data[AHT20_MEASUREMENT_SIZE-1]) {
        return ESP_ERR_AHT20_CORRUPT_DATA;
    }

    return ESP_OK;
}

// calculations as per the AHT20 datasheet
static aht20_measurement_t compute_aht20_raw_data(uint8_t* data) {
    aht20_measurement_t output;
    uint32_t raw_hum = ((uint32_t)data[1] << 12) | 
	                    ((uint32_t)data[2] << 4) | 
	                    (data[3] >> 4);

	uint32_t raw_temp = ((uint32_t)(data[3] & 0x0F) << 16) | 
	                    ((uint32_t)data[4] << 8) | 
	                    (data[5]);

    output.hum = ((float)raw_hum / 1048576.0) * 100.0;
    output.temp = ((float)raw_temp / 1048576.0) * 200.0 - 50.0;
    
    return output;
}

esp_err_t init_sensors(sensors_config_t *sensors_config) {
    i2c_master_bus_handle_t i2c_bus_handle;

    esp_err_t ret = init_i2c_port(&i2c_bus_handle);
    if(ret != ESP_OK) {
        return ret;
    }

    esp_err_t ret2 = init_bm280(&sensors_config->bmp280_handle, i2c_bus_handle, &sensors_config->bmp280_calibration);
    if(ret2 != ESP_OK) {
        return ret2;
    }

    esp_err_t ret3 = init_aht20(&sensors_config->aht20_handle, i2c_bus_handle);
    if(ret3 != ESP_OK) {
        return ret3;
    }


    return ESP_OK;
}

esp_err_t read_sensors_data(sensors_config_t sensors_config, sensor_data_t *data) {
	vTaskDelay(pdMS_TO_TICKS(100));

	uint8_t aht20_raw_data[AHT20_MEASUREMENT_SIZE];
    uint8_t bmp280_raw_data[BMP280_MEASUREMENT_SIZE];

    esp_err_t ret4 = aht20_read(sensors_config.aht20_handle, aht20_raw_data);
    if(ret4 != ESP_OK) {
        return ret4;
    }

    esp_err_t ret5 = bmp280_read(sensors_config.bmp280_handle, bmp280_raw_data);
    if(ret5 != ESP_OK) {
        return ret5;
    }

    aht20_measurement_t aht20_measurement = compute_aht20_raw_data(aht20_raw_data);
    bmp280_measurement_t bmp280_measurement = compute_bmp280_raw_data(bmp280_raw_data, sensors_config.bmp280_calibration);

    data->temperature = aht20_measurement.temp;
    data->humidity = aht20_measurement.hum;
    data->pressure = bmp280_measurement.press;

    return ESP_OK;
}
