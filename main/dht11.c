#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define TAG "DHT11"

static gpio_num_t dht_gpio;

static void dht11_set_pin_output(void) {
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
}

static void dht11_set_pin_input(void) {
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}

static void dht11_write_pin(bool level) {
    gpio_set_level(dht_gpio, level);
}

static bool dht11_read_pin(void) {
    return gpio_get_level(dht_gpio);
}

void dht11_init(gpio_num_t gpio_num) {
    dht_gpio = gpio_num;
    gpio_reset_pin(dht_gpio);
    gpio_set_pull_mode(dht_gpio, GPIO_PULLUP_ONLY);
    dht11_set_pin_output();
    dht11_write_pin(true);
}

static void delay_us(uint32_t us) {
    esp_rom_delay_us(us);  // accurate delay
}

esp_err_t dht11_read(dht11_data_t *data) {
    uint8_t bits[5] = {0};
    uint8_t byte_idx = 0, bit_idx = 7;

    // Send start signal
    dht11_set_pin_output();
    dht11_write_pin(false);
    vTaskDelay(pdMS_TO_TICKS(20));  // >18ms low
    dht11_write_pin(true);
    delay_us(30);
    dht11_set_pin_input();

    // Wait for sensor response
    int timeout = 100;
    while (dht11_read_pin()) {
        if (--timeout <= 0) return ESP_ERR_TIMEOUT;
        delay_us(1);
    }

    while (!dht11_read_pin()) delay_us(1); // LOW ~80us
    while (dht11_read_pin()) delay_us(1);  // HIGH ~80us

    // Read 40 bits
    for (int i = 0; i < 40; i++) {
        // Wait for LOW
        timeout = 100;
        while (!dht11_read_pin()) {
            if (--timeout <= 0) return ESP_ERR_TIMEOUT;
            delay_us(1);
        }

        // Measure HIGH time
        int width = 0;
        while (dht11_read_pin()) {
            delay_us(1);
            if (++width > 100) return ESP_ERR_TIMEOUT;
        }

        if (width > 40) {
            bits[byte_idx] |= (1 << bit_idx);
        }

        if (--bit_idx == 0xFF) {
            bit_idx = 7;
            byte_idx++;
        }
    }

    uint8_t checksum = bits[0] + bits[1] + bits[2] + bits[3];
    if (bits[4] != checksum) {
        ESP_LOGE(TAG, "Checksum error: expected %d, got %d", checksum, bits[4]);
        return ESP_FAIL;
    }

    data->humidity = bits[0];
    data->temperature = bits[2];
    return ESP_OK;
}
