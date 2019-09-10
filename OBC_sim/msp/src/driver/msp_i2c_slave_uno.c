/**
 * @file      msp_i2c_slave_uno.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     A minimalistic Arduino Uno I2C slave driver that is hardcoded
 *            for MSP.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#include "msp_exp.h"
#include "msp_i2c_slave.h"
#include "msp_i2c_common_uno.h"

// Buffer
static unsigned char buffer[MSP_EXP_MAX_FRAME_SIZE];
static unsigned long buffer_index;
static unsigned long buffer_length;

void msp_i2c_setup(unsigned long i2c_clockrate)
{
	msp_i2c_enable_pins();
	msp_i2c_set_clockrate(i2c_clockrate);

	// Set slave address
	TWAR = MSP_EXP_ADDR << 1;

	msp_i2c_enable_module();
	status = MSP_I2C_READY;
}

/**
 * @cond LOCAL_DEFINES
 *
 * INTERRUPT SERVICE ROUTINE
 * 
 * Set up the interrupt-vector for I2C to call this function.
 */
ISR(TWI_vect)
{
	switch (TW_STATUS) {
	/* SLAVE RECEIVE */
	case TW_SR_SLA_ACK:
	case TW_SR_GCALL_ACK:
	case TW_SR_ARB_LOST_SLA_ACK:
	case TW_SR_ARB_LOST_GCALL_ACK:
		// Slave addressed
		status = MSP_I2C_RECV;
		buffer_index = 0;
		buffer_length = 0;
		msp_i2c_signal_ack();
		break;
	case TW_SR_DATA_ACK:
	case TW_SR_GCALL_DATA_ACK:
		// Received data, ack returned
		// Ignore data if there is not enough buffer space
		if (buffer_length < MSP_EXP_MAX_FRAME_SIZE)
			buffer[buffer_length++] = TWDR;

		msp_i2c_signal_ack();
		break;
	case TW_SR_DATA_NACK:
	case TW_SR_GCALL_DATA_NACK:
		// Received data, nack returned
		msp_i2c_signal_nack();
		break;
	case TW_SR_STOP:
		// Stop condition received
		msp_recv_callback(buffer, buffer_length);
		msp_i2c_release_bus();
		break;

	/* SLAVE TRANSMIT */
	case TW_ST_SLA_ACK:
	case TW_ST_ARB_LOST_SLA_ACK:
		// Slave addressed
		status = MSP_I2C_SEND;
		msp_send_callback(buffer, &buffer_length);
		buffer_index = 0;
		/* fall through */
	case TW_ST_DATA_ACK:
		// Sent data and got ack
		// Copy data if there is more to send, else pad with 0xFF.
		if (buffer_index < buffer_length)
			TWDR = buffer[buffer_index++];
		else
			TWDR = 0xFF;

		msp_i2c_signal_ack();
		break;
	case TW_ST_DATA_NACK:
	case TW_ST_LAST_DATA:
		// Either we have received a nack or already sent the last piece of data
		msp_i2c_signal_ack();
		status = MSP_I2C_READY;
		break;

	/* THIS IS NOT EXPECTED TO BE REACHED */
	default:
		msp_i2c_release_bus();
		break;
	}
}
/** @endcond */
