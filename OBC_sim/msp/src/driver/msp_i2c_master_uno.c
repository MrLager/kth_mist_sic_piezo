/**
 * @file      msp_i2c_master_uno.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     A minimalistic I2C master implementation for Arduino Uno.
 */

#include <avr/io.h>
#include <avr/interrupt.h> 
#include <compat/twi.h>

#include "msp_i2c_master.h"
#include "msp_i2c_common_uno.h"

/** @cond LOCAL_DEFINES */
// Define the error codes
#define MSP_I2C_PARAMETER_ERROR -2
#define MSP_I2C_QUEUEING_ERROR  -1
#define MSP_I2C_WRITE_ERROR      4
#define MSP_I2C_READ_ERROR       5
#define MSP_I2C_TIMEOUT_ERROR    6
#define MSP_I2C_GENERAL_ERROR    7
/** @endcond */

// Buffer
static volatile unsigned char *buffer;
static volatile unsigned long buffer_index;
static volatile unsigned long buffer_length;

// Need to save the I2C start byte for Arduino UNO
static volatile unsigned char start_byte;

static volatile int error;

// Timeout variables
static volatile unsigned long timeout_limit;
static volatile int has_timed_out;
static volatile int reset_timeout;

static inline void msp_i2c_timeout()
{
	has_timed_out = 1;
	error = MSP_I2C_TIMEOUT_ERROR;
	msp_i2c_release_bus();
}

int msp_i2c_start(unsigned long i2c_clockrate, unsigned long i2c_timeout)
{
	status = MSP_I2C_READY;

	msp_i2c_enable_pins();
	msp_i2c_set_clockrate(i2c_clockrate);
	msp_i2c_set_timeout(i2c_timeout);

	msp_i2c_enable_module();

	return 0;
}

void msp_i2c_set_timeout(unsigned long i2c_timeout)
{
	timeout_limit = i2c_timeout;
}

void msp_i2c_stop(void)
{
	msp_i2c_disable_module();
	msp_i2c_disable_pins();
}

int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	volatile unsigned long timeout_counter;

	if (status != MSP_I2C_READY)
		return MSP_I2C_QUEUEING_ERROR; // someone is using I2C already
	if (slave_address > 0x7F)
		return MSP_I2C_PARAMETER_ERROR;
	if (size < 1)
		return MSP_I2C_PARAMETER_ERROR;

	has_timed_out = 0;
	reset_timeout = 0;
	error = 0;

	// Initialize master buffer
	buffer = data;
	buffer_index = 0;
	buffer_length = size;

	// Set up the start byte
	start_byte = (slave_address << 1) | TW_WRITE;

	// Set the I2C state of the master to that we will be transmitting to slave
	status = MSP_I2C_SEND;
	msp_i2c_signal_start();

	// Wait until the ISR is done...
	timeout_counter = 0;
	while (status == MSP_I2C_SEND && !error) {
		timeout_counter++;

		if (reset_timeout && !has_timed_out) {
			timeout_counter = 0;
			reset_timeout = 0;
		}

		if (timeout_counter >= timeout_limit)
			msp_i2c_timeout();
	}

	return error;
}

int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	volatile unsigned long timeout_counter;

	if (status != MSP_I2C_READY)
		return MSP_I2C_QUEUEING_ERROR; // someone is using I2C already
	if (slave_address > 0x7F)
		return MSP_I2C_PARAMETER_ERROR; 
	if (size < 1)
		return MSP_I2C_PARAMETER_ERROR;

	has_timed_out = 0;
	reset_timeout = 0;
	error = 0;

	// Initialize master buffer
	buffer = data;
	buffer_index = 0;
	buffer_length = size;

	// Set up start byte (is needed in the ISR)
	start_byte = (slave_address << 1) | TW_READ;

	// Set the I2C state of the master to that we will be recieving from slave
	status = MSP_I2C_RECV;
	msp_i2c_signal_start();

	// Wait until the ISR is done...
	timeout_counter = 0;
	while (status == MSP_I2C_RECV && !error) {
		timeout_counter++;

		if (reset_timeout && !has_timed_out) {
			timeout_counter = 0;
			reset_timeout = 0;
		}

		if (timeout_counter >= timeout_limit)
			msp_i2c_timeout();
	}

	return error;
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
	reset_timeout = 1;

	switch (TW_STATUS) {
	/* MASTER START */
	case TW_START:
		TWDR = start_byte; // address + R/W-bit
		msp_i2c_signal_ack();
		break;

	/* MASTER TRANSMIT */
	case TW_MT_SLA_ACK:
	case TW_MT_DATA_ACK:
		// recieved ack from slave
		if (buffer_index < buffer_length) {
			TWDR = buffer[buffer_index++];
			msp_i2c_signal_ack();
		} else {
			// we are done, signaling stop
			msp_i2c_signal_stop_and_wait();
		}
		break;
	case TW_MT_SLA_NACK:
	case TW_MT_DATA_NACK:
		// recieved nack
		error = MSP_I2C_WRITE_ERROR;
		msp_i2c_signal_stop_and_wait();
		break;

	/* MASTER RECEIVE */
	case TW_MR_SLA_ACK:
		// Received ack after sending address
		msp_i2c_signal_ack();
		break;
	case TW_MR_SLA_NACK:
		// Received nack after sending address
		error = MSP_I2C_READ_ERROR;
		msp_i2c_signal_stop_and_wait();
		break;
	case TW_MR_DATA_ACK:
		// Recieved data and ack
		if (buffer_index < buffer_length)
			buffer[buffer_index++] = TWDR;

		if (buffer_index < buffer_length)
			msp_i2c_signal_ack();
		else
			msp_i2c_signal_nack(); // Received too much data
		
		break;
	case TW_MR_DATA_NACK:
		// Recieved data and nack
		if (buffer_index < buffer_length)
			buffer[buffer_index++] = TWDR;

		if (buffer_index < buffer_length)
			error = MSP_I2C_READ_ERROR; // recieved fewer bytes than expected

		msp_i2c_signal_stop_and_wait();
		break;

	/* THIS IS NOT EXPECTED TO BE REACHED */
	default:
		error = MSP_I2C_GENERAL_ERROR;
		msp_i2c_release_bus();
		break;
	}
}
/** @endcond */
