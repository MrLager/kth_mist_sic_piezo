/**
 * @file      msp_i2c_master.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Declares necessary function prototypes for a master side I2C
 *            driver.
 *
 * @details
 * Defines functions to communicate over I2C from a master perspective. Some of
 * these functions are required to be implemented by the MSP library. It is
 * written in the prototype comments if a function is required to be
 * implemented.
 */

#ifndef MSP_I2C_MASTER_H
#define MSP_I2C_MASTER_H

/**
 * @brief Starts the I2C driver. Non-blocking call.
 * @param i2c_clockrate I2C bus speed in Hz.
 * @param i2c_timeout Timeout rate.
 * @return 0 on success, a negative value between -1 and -9 on failure.
 *
 * This function is not used by the MSP library. This function is not necessary
 * to implement if there are other ways of setting the I2C timeout.
 */
int msp_i2c_start(unsigned long i2c_clockrate, unsigned long i2c_timeout);

/**
 * @brief Sets the timeout value of the I2C driver. Non-blocking call.
 * @param i2c_timeout Timeout rate.
 *
 * This function is not used by the MSP library. This function is not necessary
 * to implement if there are other ways of setting the I2C timeout.
 */
void msp_i2c_set_timeout(unsigned long i2c_timeout);

/**
 * @brief Stops the I2C driver. Non-blocking call.
 *
 * This function is not used by the MSP library. It is not necessary to
 * implement this function if there are other ways of stopping the I2C driver.
 */
void msp_i2c_stop(void);

/**
 * @brief Writes data to a slave on the I2C bus. Blocking call. (REQUIRED)
 * @param slave_address I2C address of the slave.
 * @param data Pointer to the data to be written to the slave.
 * @param size Number of bytes from data to be written to the slave.
 * @return
 *    - 0 on success,
 *    - a value between -1 and -9 if an error occurred before initiating
 *      communication on the I2C bus, or
 *    - a value between 1 and 9 if an error occurred during communication on
 *      the I2C bus.
 *
 * REQUIRED BY THE MSP DRIVER. MUST BE IMPLEMENTED.
 *
 * Note:
 * The data pointer here should preferably be const. But to ensure that the
 * code works with other frameworks, the const is omitted here.
 */
int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size);

/**
 * @brief Reads data from a slave on the I2C bus. Blocking call. (REQUIRED)
 * @param slave_address I2C address of the slave.
 * @param data Pointer to a buffer where the received data will be stored.
 * @param size Number of bytes of data to be read from the slave.
 * @return
 *    - 0 on success,
 *    - a value between -1 and -9 if an error occurred before initiating
 *      communication on the I2C bus, or
 *    - a value between 1 and 9 if an error occurred during communication on
 *      the I2C bus.
 *
 * REQUIRED BY THE MSP DRIVER. MUST BE IMPLEMENTED.
 */
int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size);

#endif /* MSP_I2C_MASTER_H */
