/*
 * obcsim.ino
 *
 * Main file for the OBC simulator.
 */

#include "Arduino.h"
#include "msp_obc.h"
#include "obcsim_configuration.hpp"

#define I2C_SPEED (400L*1000L)
#define I2C_TIMEOUT (100L*1000L)

static msp_link_t exp_link;
static unsigned char exp_buf[EXP_MTU + 5];

/* Arduino Setup */
void setup()
{
	/* Create link to the experiment */
	exp_link = msp_create_link(EXP_ADDR, msp_seqflags_init(), exp_buf, EXP_MTU);

	/* Start I2C */
	msp_i2c_start(I2C_SPEED, I2C_TIMEOUT);

	Serial.begin(9600);
	Serial.println(F("MSP Simulator (OBC side) started."));
}

/* Arduino Loop */
void loop()
{
	static bool initiated = false;

	if (!initiated) {
		sequence_init(&exp_link);
		initiated = true;
	}
	
	sequence_loop(&exp_link);
}

