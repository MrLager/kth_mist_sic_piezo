/*
 * experiment.ino
 * Author: John Wikman
 *
 * Example program for the experiment side of MSP.
 */

extern "C" {
	#include "msp_i2c_slave.h"
}

#include "msp_exp.h"

#define I2C_SPEED (400L*1000L)

static unsigned char *send_data;
static unsigned char send_data_payload[30] = "Payload, payload and payload.";
static unsigned char send_data_hk[50] = "Housekeeping, housekeeping and housekeeping.";
static unsigned char send_data_pus[60] = "CCSDS PUS packet, CCSDS PUS packet and CCSDS PUS packet.";

static unsigned char recv_data[101];
static unsigned long recv_maxlen = 100;
static unsigned long recv_length;

static int has_send = 0;
static int has_send_error = 0;
static int has_send_errorcode = 0;
static int has_recv = 0;
static int has_recv_error = 0;
static int has_recv_errorcode = 0;
static int has_syscommand = 0;

/* Arduino Setup */
void setup()
{
	// Setup the I2C
	msp_i2c_setup(I2C_SPEED);

	// Must also initialize the experiment state
	msp_exp_state_initialize(msp_seqflags_init());

	Serial.begin(9600);
	Serial.println(F("Experiment Example started."));
}

/* Arduino Loop */
void loop()
{
	/* Send status checks */
	if (has_send) {
		Serial.print(F("[Experiment Sent Data with opcode 0x"));
		Serial.print(has_send, HEX);
		Serial.println(F("]"));
		has_send = 0;
	}
	if (has_send_error) {
		Serial.print(F("[Experiment encountered error "));
		Serial.print(has_send_errorcode, DEC);
		Serial.println(F(" when sending data with opcode 0x"));
		Serial.print(has_send_error, HEX);
		Serial.println(F("]"));
		has_send_error = 0;
	}

	/* Receive status hecks */
	if (has_recv) {
		Serial.print(F("[Experiment Received Data with opcode 0x"));
		Serial.print(has_recv, HEX);
		Serial.println(F("]"));
		Serial.println(F("----- RECEIVED DATA -----"));
		/* Assume that we got a string */
		recv_data[100] = 0;
		Serial.println((char *) recv_data);
		Serial.println(F("-------------------------"));
		has_recv = 0;
	}
	if (has_recv_error) {
		Serial.print(F("[Experiment encountered error "));
		Serial.print(has_recv_errorcode, DEC);
		Serial.println(F(" when receiving data with opcode 0x"));
		Serial.print(has_recv_error, HEX);
		Serial.println(F("]"));
		has_recv_error = 0;
	}

	/* Status checks for system commands */
	if (has_syscommand) {
		Serial.print(F("[Experiment got system command with opcode 0x"));
		Serial.print(has_syscommand, HEX);
		Serial.println(F("]"));
		has_syscommand = 0;
	}
}



/***************************************************************************
 * Below here are the implementations of the MSP functions. These could be *
 * moved to different files (and probably should be), but for this simple  *
 * example they are all in the same file.                                  *   
 ***************************************************************************/

void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	/* Determine what to send */
	if (opcode == MSP_OP_REQ_PAYLOAD) {
		send_data = send_data_payload;
		*len = sizeof(send_data_payload);
	} else if (opcode == MSP_OP_REQ_HK) {
		send_data = send_data_hk;
		*len = sizeof(send_data_hk);
	} else if (opcode == MSP_OP_REQ_PUS) {
		send_data = send_data_pus;
		*len = sizeof(send_data_pus);
	} else {
		/* If the OBC requests with an opcode that we don't care about, we just
		 * want to send 0 bytes of data. */
		*len = 0;
	}
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	for (i = 0; i < len; i++)
		buf[i] = send_data[offset + i];
}
void msp_expsend_complete(unsigned char opcode)
{
	has_send = opcode;
}
void msp_expsend_error(unsigned char opcode, int error)
{
	has_send_error = opcode;
	has_send_errorcode = error;
}


void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	/* Ignore the opcode for now. You shouldn't do this in practice though. */
	recv_length = len;
}
void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	for (i = 0; i < len; i++) {
		/* Make sure that we dont receive more than we have buffer for */
		if ((i + offset) < recv_maxlen)
			recv_data[i + offset] = buf[i];
		else
			break;
	}
}
void msp_exprecv_complete(unsigned char opcode)
{
	has_recv = opcode;
}
void msp_exprecv_error(unsigned char opcode, int error)
{
	has_recv_error = opcode;
	has_recv_errorcode = error;
}


void msp_exprecv_syscommand(unsigned char opcode)
{
	has_syscommand = opcode;
}
