#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "sensors.h"

void app_main(void)
{
    sensor_data_t sensor_data; 
    sensors_config_t sensors_config;

    ESP_ERROR_CHECK(init_sensors(&sensors_config));

    while(true) {
        ESP_ERROR_CHECK(read_sensors_data(sensors_config, &sensor_data));
        printf("Temp: %.2f°C, Hum : %.2f%%, Pressure: %.2f Pa\n", sensor_data.temperature, sensor_data.humidity, sensor_data.pressure);
        vTaskDelay(pdMS_TO_TICKS(1000));
   }
}
