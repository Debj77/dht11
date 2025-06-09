#include <stdio.h>
#include "dht11.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define DHT11_GPIO GPIO_NUM_4

void app_main(void) {
    dht11_init(DHT11_GPIO);

    while (1) {
        dht11_data_t dht;
        esp_err_t res = dht11_read(&dht);
        if (res == ESP_OK) {
            ESP_LOGI("DHT11", "Temperature: %d°C, Humidity: %d%%", dht.temperature, dht.humidity);
        } else {
            ESP_LOGE("DHT11", "Failed to read from sensor");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Wait 2 seconds
    }
}
