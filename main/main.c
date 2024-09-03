#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// sdkconfig
#include "sdkconfig.h"

// FreeRTOS
#include "FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"

// ESP api
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

// Components
#include "i2c_wrapper.h"
#include "link.h"
#include "sht3x.h"

/* Conversion factor for micro seconds to seconds */
#define US_TO_S_FACTOR 1000000ULL

/* Conversion factor for milliseconds to seconds */
#define MS_TO_S_FACTOR 1000UL

#define TAG "MAIN"

void init_nvs(void);
char *on_link_data_message(void);
void sleep_esp(bool is_error);

link_config_t my_device = {
    .type = 1,
    .config = "\"data_interval\": 300",
    .data_fmt = "{\"T\":%.2f,\"H\":%.2f}",
    .user_data_msg_cb = on_link_data_message,
};

void app_main() {
  init_nvs();
  uart_set_baudrate(UART_NUM_0, 115200);

  // GPIO init
  gpio_set_direction(CONFIG_GPIO_LED, GPIO_MODE_OUTPUT);
  gpio_set_level(CONFIG_GPIO_LED, CONFIG_GPIO_LED_ACTIVE_STATE);

  // Button init
  gpio_config_t button_cfg;
  button_cfg.pin_bit_mask = (1UL << CONFIG_GPIO_BUTTON);
  button_cfg.mode = GPIO_MODE_INPUT;
  button_cfg.pull_up_en = (CONFIG_GPIO_BUTTON_ACTIVE_STATE)
                              ? GPIO_PULLUP_DISABLE
                              : GPIO_PULLUP_ENABLE;
  button_cfg.pull_down_en = (CONFIG_GPIO_BUTTON_ACTIVE_STATE)
                                ? GPIO_PULLDOWN_ENABLE
                                : GPIO_PULLDOWN_DISABLE;
  gpio_config(&button_cfg);

  // I2C init
  i2c_init(I2C_NUM_0, CONFIG_GPIO_I2C_SCL, CONFIG_GPIO_I2C_SDA);

  // Check if button is pressed
  vTaskDelay(pdMS_TO_TICKS(10));
  bool is_force_pair_btn_pressed =
      (gpio_get_level(CONFIG_GPIO_BUTTON) == CONFIG_GPIO_BUTTON_ACTIVE_STATE);

  // Link init
  link_register(&my_device);
  link_start(is_force_pair_btn_pressed);

  // Wait until Link finds a pair
  link_block_until_find_pair();

  // Send data
  bool is_received = link_send_data_msg();

  // Turn off led
  gpio_set_level(CONFIG_GPIO_LED, !CONFIG_GPIO_LED_ACTIVE_STATE);

  // Go to sleep
  sleep_esp(!is_received);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void init_nvs(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

char *on_link_data_message(void) {
  float temperature, humidity;
  esp_err_t err = sht3x_read_temperature_humidity(&temperature, &humidity);

  if (err != ESP_OK) {
    sleep_esp(true);
  }

  char *status = link_generate_data_message(NULL, temperature, humidity);
  return status;
}

void sleep_esp(bool is_error) {
  const uint64_t time_s =
      (is_error ? CONFIG_DEEP_SLEEP_DURATION_NO_DATA_OR_ERROR
                : CONFIG_DEEP_SLEEP_DURATION_DATA_RECEIVED) -
      (esp_random() % CONFIG_DEEP_SLEEP_RANDOM_TIME_OFFSET);

#if CONFIG_USE_DEEP_SLEEP
  ESP_LOGI(TAG, "Going into deep sleep for %llu seconds", time_s);
  esp_deep_sleep(time_s * US_TO_S_FACTOR);
#else
  ESP_LOGI(TAG, "Waiting for %llu seconds", time_s);
  vTaskDelay(pdMS_TO_TICKS(time_s * MS_TO_S_FACTOR));
  esp_restart();
#endif
}