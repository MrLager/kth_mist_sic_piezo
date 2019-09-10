/**
 * @file      msp_i2c_slave_due.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     A minimalistic Arduino Due I2C slave driver that is hardcoded
 *            for MSP.
 */

/*
 * Mnemonics for defines in Atmel's TWI driver:
 *  - TWI_SR_<...> status register bit mask
 *  - TWI_IER_<...> enable interrupt bit mask
 *  - TWI_IDR_<...> disable interrupt bit mask
 */

#include <include/twi.h> /* Atmel TWI driver */
#include "variant.h" /* WIRE pins and definitions */

#include "msp_exp.h"
#include "msp_i2c_slave.h"
#include "msp_i2c_common_due.h" /* defines the variable 'twi' */

/** @cond LOCAL_DEFINES */
/* The I2C state */
typedef enum {
	MSP_I2C_UNINITIALIZED = 0,
	MSP_I2C_READY = 1,
	MSP_I2C_RECV = 2,
	MSP_I2C_SEND = 3
} msp_i2c_state_t;
/** @endcond */

static msp_i2c_state_t status = MSP_I2C_UNINITIALIZED;

// Buffer
static unsigned char buffer[MSP_EXP_MAX_FRAME_SIZE];
static unsigned long buffer_index;
static unsigned long buffer_length;

/* Configures the ISR to only get triggered when addressed */
static inline void msp_i2c_isr_ready(void)
{
	TWI_DisableIt(twi, TWI_IDR_TXCOMP | TWI_IDR_RXRDY | TWI_IDR_GACC |
	                   TWI_IDR_NACK | TWI_IDR_SCL_WS | TWI_IDR_EOSACC);
	TWI_EnableIt(twi, TWI_IER_SVACC);
}

/* Configures the ISR to only trigger on I2C transaction events */
static inline void msp_i2c_isr_busy(void)
{
	TWI_DisableIt(twi, TWI_IDR_SVACC);
	TWI_EnableIt(twi, TWI_IER_TXCOMP | TWI_IER_RXRDY | TWI_IER_GACC |
	                  TWI_IER_NACK | TWI_IER_SCL_WS | TWI_IER_EOSACC);
}

void msp_i2c_setup(unsigned long i2c_clockrate)
{
	msp_i2c_enable();
	status = MSP_I2C_READY;

	TWI_ConfigureSlave(twi, MSP_EXP_ADDR);
	msp_i2c_isr_ready();
}

/**
 * @cond LOCAL_DEFINES
 *
 * I2C ISR that is hardcoded to use the MSP callback functions. 
 */
void WIRE_ISR_HANDLER(void)
{
	// Get TWI status
	unsigned long sr = TWI_GetStatus(twi);

	/* Transmission Begin (slave addressed) */
	if (status == MSP_I2C_READY && TWI_STATUS_SVACC(sr)) {
		msp_i2c_isr_busy();

		buffer_index = 0;
		buffer_length = 0;

		if (TWI_STATUS_SVREAD(sr)) {
			/* Send data to Master */
			msp_send_callback(buffer, &buffer_length);
			status = MSP_I2C_SEND;
		} else {
			/* Receive data from Master */
			status = MSP_I2C_RECV;
		}
	}

	/* Transmission End (end of slave access) */
	if (status != MSP_I2C_READY && TWI_STATUS_EOSACC(sr)) {
		if (status == MSP_I2C_RECV)
			msp_recv_callback(buffer, buffer_length);

		msp_i2c_isr_ready();
		status = MSP_I2C_READY;
	}

	/* Receive Byte */
	if (status == MSP_I2C_RECV && TWI_STATUS_RXRDY(sr)) {
		if (buffer_length < MSP_EXP_MAX_FRAME_SIZE)
			buffer[buffer_length++] = TWI_ReadByte(twi);
	}

	/* Send Byte */
	if (status == MSP_I2C_SEND && TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
		if (buffer_index < buffer_length)
			TWI_WriteByte(twi, buffer[buffer_index++]);
		else
			TWI_WriteByte(twi, 0xFF); /* padding as specified by MSP */
	}
}
/** @endcond */
