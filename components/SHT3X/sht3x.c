// sdkconfig
#include "../../build/include/sdkconfig.h"

// FreeRTOS
#include "FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "portmacro.h"

// ESP api
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"

// other
#include "stdbool.h"
#include "string.h"

// -------
#include "i2c_wrapper.h"
#include "sht3x.h"
#include "sht3x_crc.h"

#define TAG "SHT3X"

esp_err_t sht3x_read_temperature_humidity(float* temperature, float* humidity) {
  esp_err_t esp_err;
  uint8_t readbuffer[6];

  // send measurement request
  esp_err = i2c_write_word_to_dev(I2C_NUM_0, SHT3X_DEFAULT_ADDR, SHT3X_MEAS_HIGHREP);

  if (esp_err != ESP_OK) {
    ESP_LOGE(TAG, "Sending command %04X FAILED - esp_err_t code: 0x%.2X", SHT3X_MEAS_HIGHREP, esp_err);
    return esp_err;
  }

  // wait for measurement
  vTaskDelay(pdMS_TO_TICKS(20));

  // read
  esp_err = i2c_dev_read(
      I2C_NUM_0,
      SHT3X_DEFAULT_ADDR,
      NULL,  // no need to write address
      0,
      readbuffer,
      sizeof(readbuffer));

  if (esp_err != ESP_OK) {
    ESP_LOGE(TAG, "Reading from sht3x failed - esp_err_t code: 0x%.2X", esp_err);
    return esp_err;
  }

  // crc
  if (crc8(readbuffer, 2) != readbuffer[2]) {
    ESP_LOGE(TAG, "CRC check for temperature data failed");
    return ESP_ERR_INVALID_CRC;
  }


  if (crc8(readbuffer + 3, 2) != readbuffer[5]) {
    ESP_LOGE(TAG, "CRC check for humidity data failed");
    return ESP_ERR_INVALID_CRC;
  }

  // compute values
  *temperature = ((((readbuffer[0] * 256.0) + readbuffer[1]) * 175) / 65535.0) - 45;
  *humidity = ((((readbuffer[3] * 256.0) + readbuffer[4]) * 100) / 65535.0);

  return ESP_OK;
}
