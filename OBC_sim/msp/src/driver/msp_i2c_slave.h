/**
 * @file      msp_i2c_slave.h
 * @author    Johan Sjöblom
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Declares a function for setting up an I2C slave driver in MSP.
 *
 * @details
 * Declares a function prototype that can be used for setting up an I2C slave
 * driver in MSP. The driver that implements this function should be hardcoded
 * for MSP and use the experiment callback functions.
 */

#ifndef MSP_I2C_SLAVE_H
#define MSP_I2C_SLAVE_H

/**
 * @brief This must be called before any I2C transmissions can start.
 * @param i2c_clockrate I2C bus speed in Hz.
 */
void msp_i2c_setup(unsigned long i2c_clockrate);

#endif /* MSP_I2C_SLAVE_H */
