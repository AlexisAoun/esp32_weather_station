#include <stdint.h>
#include <stdio.h>
#include <driver/i2c_master.h>
#include <stdlib.h>
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"

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

//BMP180
#define BMP280_DEVICE_ADDRESS 0x77
#define BMP280_SCL_SPEEH_HZ 100000
#define BMP280_ID_REGISTER_ADDRESS 0xD0

typedef struct {
    float temp;
    float hum;
} Aht20_measurement;

static void init_i2c_port(i2c_master_bus_handle_t* bus_handle) {
	i2c_master_bus_config_t i2c_mst_config = {
	    .clk_source = I2C_CLK_SRC_DEFAULT,
	    .i2c_port = I2C_PORT,
	    .scl_io_num = SCL_IO_NUM,
	    .sda_io_num = SDA_IO_NUM,
	    .glitch_ignore_cnt = GLITCH_IGNORE_CNT,
	    .flags.enable_internal_pullup = ENABLE_INTERNAL_PULLUP,
	};

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, bus_handle));
}

static void init_bm280(i2c_master_dev_handle_t* dev_handle, i2c_master_bus_handle_t bus_handle) {
	i2c_device_config_t dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = BMP280_DEVICE_ADDRESS,
	    .scl_speed_hz = BMP280_SCL_SPEEH_HZ,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle));

	vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t id[1];
    uint8_t id_reg = 0xD0;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &id_reg, 1, id, 1, -1));
    //ESP_ERROR_CHECK(i2c_master_receive(*dev_handle, id, 1, -1));

    for(int i = 0; i < 1; i++) {
        printf("id is : 0x%02X \n", id[i]);
    }

    uint8_t config[] = {0xF4, 0x93};
    //ESP_ERROR_CHECK(i2c_master_transmit(*dev_handle, &config_reg, 1, -1));
    ESP_ERROR_CHECK(i2c_master_transmit(*dev_handle, config, 2, -1));
	vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t press_reg = 0xF7;
    uint8_t raw_data[6];
    size_t raw_data_length = 6;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(*dev_handle, &press_reg, 1, raw_data, raw_data_length, -1));

    printf("raw data : \n");
    for(int i = 0; i < raw_data_length; i++) {
        printf("0x%02X ", raw_data[i]);
    }
    printf("\n");
}

static void init_aht20(i2c_master_dev_handle_t* dev_handle, i2c_master_bus_handle_t bus_handle) {
	i2c_device_config_t dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = AHT20_DEVICE_ADDRESS,
	    .scl_speed_hz = AHT20_SCL_SPEEH_HZ,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle));
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

void aht20_read(i2c_master_dev_handle_t dev_handle, uint8_t* data, size_t len) {
	uint8_t command[3] = AHT20_MEASUREMENT_COMMAND;
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, command, 3, -1));
	
	uint8_t status; 
	int timeout = 10;

    // AHT20 is ready for measurement when bit[7] of the status word is 0
    do {
	    vTaskDelay(pdMS_TO_TICKS(80)); 
	    i2c_master_receive(dev_handle, &status, 1, -1);
	    timeout--;
	} while ((status & 1) && timeout > 0);
	
	if (timeout == 0) {
	    printf("Sensor timeout / Busy\n");
        //TODO how to handle errors in esp
	} 

	ESP_ERROR_CHECK(i2c_master_receive(dev_handle, data, len, -1));

    //CRC is the last byte
	uint8_t crc = aht20_calc_crc(data, len-1);

    if (crc != data[len-1]) {
	    printf("Corrupt data\n");
        //TODO how to handle errors in esp
    }
}

Aht20_measurement compute_aht20_raw_data(uint8_t* data) {
    // calculations as per the AHT20 datasheet
    Aht20_measurement output;
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


void app_main(void)
{
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t aht20_handle;
    i2c_master_dev_handle_t bmp280_handle;

    init_i2c_port(&i2c_bus_handle);
    init_bm280(&bmp280_handle, i2c_bus_handle);
 //    init_aht20(&aht20_handle, i2c_bus_handle);
	//
	// vTaskDelay(pdMS_TO_TICKS(100));
	//
	// while(true) {
	//     uint8_t aht20_raw_data[AHT20_MEASUREMENT_SIZE];
 //        aht20_read(aht20_handle, aht20_raw_data, AHT20_MEASUREMENT_SIZE);
	//
 //        Aht20_measurement aht20_measurement = compute_aht20_raw_data(aht20_raw_data);
	//     printf("Temp: %.2f°C, Humidity: %.2f%%\n", aht20_measurement.temp, aht20_measurement.hum);
	//
	//     vTaskDelay(pdMS_TO_TICKS(1000));
	// }
}
