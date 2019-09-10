/**
 * @file      msp_i2c_master_iobc.c
 * @author    Muhamad Andung Muntaha
 * @copyright MIT License
 * @brief     I2C master interface towards the ISISpace iOBC HAL.
 *
 * @details
 * This driver requires the HAL to be on the include path. Specifically, the
 * statement "#include <hal/Drivers/I2C.h>" should work without errors.
 */

#include "msp_i2c_master.h"

#include <hal/Drivers/I2C.h>


/* Start the I2C driver */
int msp_i2c_start(unsigned long i2c_clockrate, unsigned long i2c_timeout)
{
	return I2C_start(i2c_clockrate, i2c_timeout);
}

/* Set I2C timeout */
void msp_i2c_set_timeout(unsigned long i2c_timeout)
{
	I2C_setTransferTimeout(i2c_timeout);
}

/* Stop the I2C driver */
void msp_i2c_stop(void)
{
	I2C_stop();
}

/* Write over I2C to the specified address */
int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	return I2C_write(slave_address, data, size);
}

/* Read over I2C from the specified address */
int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	return I2C_read(slave_address, data, size);
}
