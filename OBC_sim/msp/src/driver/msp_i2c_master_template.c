/**
 * @file      msp_i2c_master_template.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     A template file for I2C master drivers.
 *
 * @details
 * A template file for implementing an I2C driver towards the OBC side of
 * MSP.
 */

/* Start the I2C driver */
int msp_i2c_start(unsigned long i2c_clockrate, unsigned long i2c_timeout)
{
	/* This function is not used by the MSP driver, so implementing this
	 * function is optional. */
	return -1;
}

/* Set I2C timeout */
void msp_i2c_set_timeout(unsigned long i2c_timeout)
{
	/* This function is not used by the MSP driver, so implementing this
	 * function is optional. */
}

/* Stop the I2C driver */
void msp_i2c_stop(void)
{
	/* This function is not used by the MSP driver, so implementing this
	 * function is optional. */
}

/* Write over I2C to the specified address */
int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	/* Required by the MSP driver */
	#error msp_i2c_write not implemented
}

/* Read over I2C from the specified address */
int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	/* Required by the MSP driver */
	#error msp_i2c_read not implemented
}
