/**
 * @file      msp_i2c_common_uno.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Common definitions for the Arduino Uno I2C drivers in MSP.
 *
 * @details
 * Common definitions used by both the slave and master I2C implementation on
 * Arduino Uno.
 */

#ifndef MSP_I2C_COMMON_UNO_H
#define MSP_I2C_COMMON_UNO_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include "Arduino.h"
#include "pins_arduino.h"

/**
 * @brief An enum to determine the I2C state on an Arduino Uno.
 *
 * This is used by the master and the slave since both of them operate through
 * an ISR.
 */
typedef enum {
	MSP_I2C_UNINITIALIZED = 0, /**< I2C driver is uninitialized. */
	MSP_I2C_READY = 1, /**< I2C is ready/idle.  */
	MSP_I2C_RECV = 2, /**< Receiving data over I2C */
	MSP_I2C_SEND = 3 /**< Sending data over I2C */
} msp_i2c_state_t;

/* Declare the status variable */
static volatile msp_i2c_state_t status = MSP_I2C_UNINITIALIZED;


/************************************************************
 * Static Inline functions used by both slave and/or master *
 ************************************************************/

/**
 * @brief Bit Mask for TWCR that is common for all I2C signals
 */
#define SIGNAL_BV (_BV(TWEN) | _BV(TWIE) | _BV(TWINT))

static inline void msp_i2c_signal_ack(void)
{
	TWCR = SIGNAL_BV | _BV(TWEA);
}

static inline void msp_i2c_signal_nack(void)
{
	/* Unlike other signals, we do not want to generate ACK pulse here */
	TWCR = SIGNAL_BV;
}

static inline void msp_i2c_signal_start(void)
{
	TWCR = SIGNAL_BV | _BV(TWEA) | _BV(TWSTA);
}

static inline void msp_i2c_signal_stop_and_wait(void)
{
	/* Signal stop condition */
	TWCR = SIGNAL_BV | _BV(TWEA) | _BV(TWSTO);

	/* Wait for it to be sent before updating status */
	while (TWCR & _BV(TWSTO));

	status = MSP_I2C_READY;
}

static inline void msp_i2c_release_bus(void)
{
	TWCR = SIGNAL_BV | _BV(TWEA);

	status = MSP_I2C_READY;
}

static inline void msp_i2c_enable_module(void)
{
	/* No TWINT flag here as we don't want to generate an interrupt */
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

static inline void msp_i2c_disable_module(void)
{
	/* Set all known flags to 0 (except for TWWC) */
	TWCR &= ~(SIGNAL_BV | _BV(TWSTA) | _BV(TWSTO) | _BV(TWEA));
}

static inline void msp_i2c_enable_pins(void)
{
	digitalWrite(SDA, HIGH);
	digitalWrite(SCL, HIGH);
}

static inline void msp_i2c_disable_pins(void)
{
	digitalWrite(SDA, LOW);
	digitalWrite(SCL, LOW);
}

static inline void msp_i2c_set_clockrate(unsigned long i2c_clockrate)
{
	/* From the ATmega328P specification:
	 *
	 * SCL Frequency = CPU Clock Frequency / (16 + 2*TWBR*PrescalerValue)
	 * 
	 * this gives that
	 * TWBR = ((CPU Clock Frequency / SCL Frequency) - 16) / (2*PrescalerValue)
	 */

	// Set prescaler value to 1 (TWPS0 = 0 and TWPS1 = 0)
	TWSR &= ~_BV(TWPS0);
	TWSR &= ~_BV(TWPS1);

	// Arduino formula for calculating I2C bitrate
	TWBR = ((F_CPU / i2c_clockrate) - 16) / (2*1);
}

#endif /* MSP_I2C_COMMON_UNO_H */
