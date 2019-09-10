/**
 * @file      msp_i2c_master_due.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     A minimalistic I2C master implementation for Arduino Due.
 */

#include <include/twi.h> /* Atmel TWI Driver */
#include "variant.h" /* WIRE pins and definitions */

#include "msp_i2c_master.h"
#include "msp_i2c_common_due.h" /* defines the variable 'twi' */

/** @cond LOCAL_DEFINES */
/* Define the error codes */
#define MSP_I2C_PARAMETER_ERROR -2
#define MSP_I2C_WRITE_ERROR      4
#define MSP_I2C_READ_ERROR       5
#define MSP_I2C_TIMEOUT_ERROR    6
#define MSP_I2C_GENERAL_ERROR    7
/** @endcond */

static unsigned long timeout_limit = 0;

/*
 * Wait until the given status is set. If a nack occurrs during waiting, the
 * specified nack_error_code is returned instead. If a timeout value is set it
 * will return a timeout error on timeout.
 */
static inline int msp_i2c_wait_for(int twi_status, int nack_error_code)
{
	volatile unsigned long timeout = timeout_limit;
	unsigned long sr = 0;

	while (!TWI_STATUS_ISSET(sr, twi_status)) {
		sr = TWI_GetStatus(twi);

		if (TWI_STATUS_NACK(sr))
			return nack_error_code;

		if (timeout_limit) {
			timeout--;
			if (timeout == 0)
				return MSP_I2C_TIMEOUT_ERROR;
		}
	}

	return 0; //OK
}
/* Standard wait functions */
static int msp_i2c_wait_for_send()
{
	return msp_i2c_wait_for(TWI_SR_TXRDY, MSP_I2C_WRITE_ERROR);
}
static int msp_i2c_wait_for_recv()
{
	return msp_i2c_wait_for(TWI_SR_RXRDY, MSP_I2C_READ_ERROR);
}
static int msp_i2c_wait_for_complete()
{
	return msp_i2c_wait_for(TWI_SR_TXCOMP, MSP_I2C_GENERAL_ERROR);
}

int msp_i2c_start(unsigned long i2c_clockrate, unsigned long i2c_timeout)
{
	msp_i2c_enable();
	msp_i2c_set_timeout(i2c_timeout);

	TWI_ConfigureMaster(twi, i2c_clockrate, VARIANT_MCK);

	return 0;
}

void msp_i2c_set_timeout(unsigned long i2c_timeout)
{
	timeout_limit = i2c_timeout;
}

void msp_i2c_stop(void)
{
	TWI_Disable(twi);
	
	msp_i2c_disable();
}

/*
 * Writes to a slave.
 */
int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	if (slave_address > 0x7F)
		return MSP_I2C_PARAMETER_ERROR;
	if (size < 1)
		return MSP_I2C_PARAMETER_ERROR;

	int error = 0;

	TWI_StartWrite(twi, (unsigned char) slave_address, 0, 0, data[0]);

	error = msp_i2c_wait_for_send();

	unsigned long i = 1;
	while (i < size && !error) {
		TWI_WriteByte(twi, data[i++]);
		error = msp_i2c_wait_for_send();
	}

	TWI_Stop(twi);
	if (error) // dont overwrite error code if we have one
		msp_i2c_wait_for_complete();
	else
		error = msp_i2c_wait_for_complete();

	return error;
}

/*
 * Reads from a slave.
 */
int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	if (slave_address > 0x7F)
		return MSP_I2C_PARAMETER_ERROR;
	if (size < 1)
		return MSP_I2C_PARAMETER_ERROR;

	int error = 0;

	TWI_StartRead(twi, (unsigned char) slave_address, 0, 0);

	unsigned long i = 0;
	while (i < size && !error) {
		// Send stop condition at last byte
		if (i + 1 == size)
			TWI_SendSTOPCondition(twi);

		error = msp_i2c_wait_for_recv();
		if (error == 0)
			data[i++] = TWI_ReadByte(twi);
	}

	if (error) // dont overwrite error code if we have one
		msp_i2c_wait_for_complete();
	else
		error = msp_i2c_wait_for_complete();

	return error;
}
