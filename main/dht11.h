#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

typedef struct {
    uint8_t temperature;
    uint8_t humidity;
} dht11_data_t;

/**
 * @brief Initialize DHT11 GPIO.
 * @param gpio_num GPIO used for DHT11 data line.
 */
void dht11_init(gpio_num_t gpio_num);

/**
 * @brief Read data from DHT11 sensor.
 * @param[out] data Pointer to store temperature and humidity.
 * @return ESP_OK on success, ESP_FAIL or ESP_ERR_TIMEOUT on failure.
 */
esp_err_t dht11_read(dht11_data_t *data);

#endif
