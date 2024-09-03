#ifndef COMPONENTS_I2C_WRAPPER_H_
#define COMPONENTS_I2C_WRAPPER_H_

#include "esp_err.h"

// Mutex calls with the Return function 
#define I2C_TAKE_MUTEX                  \
  do {                                  \
    esp_err_t _err_ = i2c_take_mutex(); \
    if (_err_ != ESP_OK) return _err_;  \
  } while (0)

#define I2C_GIVE_MUTEX                  \
  do {                                  \
    esp_err_t _err_ = i2c_give_mutex(); \
    if (_err_ != ESP_OK) return _err_;  \
  } while (0)

// Mutex calls without the Return function
#define I2C_TAKE_MUTEX_NORET \
  do {                       \
    i2c_take_mutex();        \
  } while (0)

#define I2C_GIVE_MUTEX_NORET \
  do {                       \
    i2c_give_mutex();        \
  } while (0)

extern esp_err_t i2c_take_mutex(void);
extern esp_err_t i2c_give_mutex(void);

extern void force_i2c_stop(uint8_t on);

/**
 * @brief Initialize the I2C driver.
 *
 * This function configures and installs the I2C driver for the specified port number.
 *
 * @param[in] port_nr I2C port number to initialize (typically 0 or 1). On ESP8266 is only 0
 * @param[in] scl_pin_nr GPIO pin number to use for the SCL (clock) line.
 * @param[in] sda_pin_nr GPIO pin number to use for the SDA (data) line.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Parameter error
 *     - ESP_FAIL: Driver installation failed
 *
 * @note This function assumes the use of pull-up resistors on both the SDA and SCL lines.
 *       The clock stretch value is set to 300 ticks, which is approximately 210us. Adjust
 *       this value according to the actual requirements of your application.
 */
extern esp_err_t i2c_init(uint8_t port_nr, uint8_t scl_pin_nr, uint8_t sda_pin_nr);

/**
 * @brief Check the presence of an I2C device at the specified address.
 *
 * This function sends a start condition followed by the I2C address of the slave device.
 * If the device acknowledges, the function will return ESP_OK, indicating that the device
 * is present and responding on the I2C bus.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to check.
 *
 * @return
 *     - ESP_OK: Success, the device is present
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_check_dev(uint8_t port_nr, uint8_t slave_addr);

/**
 * @brief Write a single byte to a specified I2C device.
 *
 * This function sends a start condition, followed by the I2C address of the slave device,
 * and then writes a single byte of data. Finally, a stop condition is sent to complete
 * the transaction.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to write to.
 * @param[in] byte The data byte to write to the slave device.
 *
 * @return
 *     - ESP_OK: Success, the byte was written
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_write_byte_to_dev(uint8_t port_nr, uint8_t slave_addr, uint8_t byte);

/**
 * @brief Write a 16-bit word to a specified I2C device.
 *
 * This function sends a start condition, followed by the I2C address of the slave device,
 * and then writes a 16-bit word (two bytes) of data. The function writes the most significant 
 * byte (MSB) first, followed by the least significant byte (LSB). Finally, a stop condition is 
 * sent to complete the transaction.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to write to.
 * @param[in] word The 16-bit word to write to the slave device.
 *
 * @return
 *     - ESP_OK: Success, the word was written
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_write_word_to_dev(uint8_t port_nr, uint8_t slave_addr, uint16_t word);

/**
 * @brief Read a single byte from a specified I2C device.
 *
 * This function sends a start condition, followed by the I2C address of the slave device
 * with the read bit set. It then reads a single byte of data from the slave device. Finally,
 * a stop condition is sent to complete the transaction.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to read from.
 * @param[out] byte Pointer to a variable where the read byte will be stored.
 *
 * @return
 *     - ESP_OK: Success, the byte was read
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_read_byte_from_dev(uint8_t port_nr, uint8_t slave_addr, uint8_t *byte);

/**
 * @brief Read a 16-bit word from a specified I2C device.
 *
 * This function sends a start condition, followed by the I2C address of the slave device
 * with the read bit set. It then reads two bytes of data from the slave device: the most
 * significant byte (MSB) first, followed by the least significant byte (LSB). Finally,
 * a stop condition is sent to complete the transaction. The read bytes are combined into
 * a 16-bit word.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to read from.
 * @param[out] word Pointer to a variable where the read 16-bit word will be stored.
 *
 * @return
 *     - ESP_OK: Success, the word was read
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_read_word_from_dev(uint8_t port_nr, uint8_t slave_addr, uint16_t *word);

/**
 * @brief Perform an I2C transaction with a specified device, sending data and then reading a response.
 *
 * This function performs an I2C transaction with the specified slave device. It can first send a block
 * of data to the device and then read a block of data from the device. If no data needs to be sent, the
 * function can be used to only read data from the device.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to communicate with.
 * @param[in] out_data Pointer to the data to send to the device (can be NULL if no data is to be sent).
 * @param[in] out_size Size of the data to send to the device (can be 0 if no data is to be sent).
 * @param[out] in_data Pointer to a buffer where the read data will be stored.
 * @param[in] in_size Size of the buffer to store the read data.
 *
 * @return
 *     - ESP_OK: Success, the transaction was completed
 *     - ESP_ERR_INVALID_ARG: Invalid argument (e.g., in_data is NULL or in_size is 0)
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_dev_read(uint8_t port_nr, uint8_t slave_addr,
                              const void *out_data, size_t out_size,
                              void *in_data, size_t in_size);

/**
 * @brief Perform an I2C transaction to write data to a specified device, optionally writing to a register.
 *
 * This function performs an I2C transaction with the specified slave device. It can write a block
 * of data to a specific register of the device. If no register needs to be specified, the function
 * can be used to only write data to the device.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to communicate with.
 * @param[in] out_reg Pointer to the register address to write to (can be NULL if no register is to be specified).
 * @param[in] out_reg_size Size of the register address (can be 0 if no register is to be specified).
 * @param[in] out_data Pointer to the data to write to the device.
 * @param[in] out_size Size of the data to write to the device.
 *
 * @return
 *     - ESP_OK: Success, the transaction was completed
 *     - ESP_ERR_INVALID_ARG: Invalid argument (e.g., out_data is NULL or out_size is 0)
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function uses a mutex to ensure that the I2C bus is not accessed concurrently
 *       by multiple tasks. If the force_i2c_stop_flag is set, it will forcibly stop the I2C
 *       operation.
 */
extern esp_err_t i2c_dev_write(uint8_t port_nr, uint8_t slave_addr,
                               const void *out_reg, size_t out_reg_size,
                               const void *out_data, size_t out_size);

/**
 * @brief Read data from a specified register of an I2C device.
 *
 * This function reads a block of data from a specific register of the specified slave device.
 * It uses the `i2c_dev_read` function to perform the transaction.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to read from.
 * @param[in] reg The register address to read from.
 * @param[out] in_data Pointer to a buffer where the read data will be stored.
 * @param[in] in_size Size of the buffer to store the read data.
 *
 * @return
 *     - ESP_OK: Success, the data was read
 *     - ESP_ERR_INVALID_ARG: Invalid argument (e.g., in_data is NULL or in_size is 0)
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function is a convenience wrapper around `i2c_dev_read` to simplify reading from a specific register.
 */
extern esp_err_t i2c_dev_read_reg(uint8_t port_nr, uint8_t slave_addr, uint8_t reg,
                                  void *in_data, size_t in_size);

/**
 * @brief Write data to a specified register of an I2C device.
 *
 * This function writes a block of data to a specific register of the specified slave device.
 * It uses the `i2c_dev_write` function to perform the transaction.
 *
 * @param[in] port_nr I2C port number to use for the operation (typically 0 or 1).
 * @param[in] slave_addr I2C address of the slave device to write to.
 * @param[in] reg The register address to write to.
 * @param[in] out_data Pointer to the data to write to the device.
 * @param[in] out_size Size of the data to write to the device.
 *
 * @return
 *     - ESP_OK: Success, the data was written
 *     - ESP_ERR_INVALID_ARG: Invalid argument (e.g., out_data is NULL or out_size is 0)
 *     - ESP_ERR_TIMEOUT: Operation timed out
 *     - ESP_FAIL: I2C driver error
 *
 * @note This function is a convenience wrapper around `i2c_dev_write` to simplify writing to a specific register.
 */
extern esp_err_t i2c_dev_write_reg(uint8_t port_nr, uint8_t slave_addr, uint8_t reg,
                                   const void *out_data, size_t out_size);

#endif  // COMPONENTS_I2C_WRAPPER_H_