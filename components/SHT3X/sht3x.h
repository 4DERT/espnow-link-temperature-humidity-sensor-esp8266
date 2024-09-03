#ifndef SHT3X_H_
#define SHT3X_H_

#define SHT3X_DEFAULT_ADDR 0x8A           /**< SHT3X Default Address */
#define SHT3X_MEAS_HIGHREP_STRETCH 0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT3X_MEAS_MEDREP_STRETCH 0x2C0D  /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT3X_MEAS_LOWREP_STRETCH 0x2C10  /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT3X_MEAS_HIGHREP 0x2400         /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT3X_MEAS_MEDREP 0x240B          /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT3X_MEAS_LOWREP 0x2416          /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT3X_READSTATUS 0xF32D           /**< Read Out of Status Register */
#define SHT3X_CLEARSTATUS 0x3041          /**< Clear Status */
#define SHT3X_SOFTRESET 0x30A2            /**< Soft Reset */
#define SHT3X_HEATEREN 0x306D             /**< Heater Enable */
#define SHT3X_HEATERDIS 0x3066            /**< Heater Disable */
#define SHT3X_REG_HEATER_BIT 0x0d         /**< Status Register Heater Bit */

// extern void sht3x_reset(void);

/**
 * @brief Read temperature and humidity from the SHT3x sensor.
 *
 * This function sends a measurement request to the SHT3x sensor, waits for the measurement
 * to complete, and then reads the temperature and humidity data from the sensor. The read
 * data is checked for integrity using CRC, and the temperature and humidity values are computed.
 *
 * @param[out] temperature Pointer to a variable where the read temperature will be stored.
 * @param[out] humidity Pointer to a variable where the read humidity will be stored.
 *
 * @return
 *     - ESP_OK: Success, the temperature and humidity were read and computed correctly
 *     - ESP_ERR_INVALID_CRC: CRC check failed for the read data
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses the I2C bus to communicate with the SHT3x sensor. Ensure that the
 *       I2C bus and the sensor are correctly configured and connected.
 */
extern esp_err_t sht3x_read_temperature_humidity(float* temperature, float* humidity);


#endif  // SHT3X_H_