/*
 * obc.ino
 * Author: John Wikman
 *
 * Example program for the OBC side of MSP.
 */

#include "msp_obc.h"

#define MSP_MTU 15

#define I2C_SPEED (400L*1000L)
#define I2C_TIMEOUT (100L*1000L)

static msp_link_t test_link;
static unsigned char test_buf[MSP_MTU+5];

static unsigned char test_recv[101];

static unsigned char test_send_pus[30] = "Here's a CCSDS PUS packet!";
static unsigned char test_send_time[4] = "clk";
static unsigned char *test_send;
static unsigned long test_send_size;

static void send_data(unsigned char opcode, const char *name);
static void receive_data(unsigned char opcode, const char *name);
static void send_syscommand(unsigned char opcode, const char *name);

static struct msp_response process(msp_link_t *lnk);
static void print_response(struct msp_response r, const char *msg);

/* Arduino Setup */
void setup()
{
	/* Create link to experiment with address 0x11 */
	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, MSP_MTU);

	/* Start I2C */
	msp_i2c_start(I2C_SPEED, I2C_TIMEOUT);

	Serial.begin(9600);
	Serial.println(F("OBC Example started."));
}

/* Arduino Loop */
void loop()
{
	static int transaction_type = 0;

	/* setup send types */
	unsigned char send_opcodes[2] = {MSP_OP_SEND_PUS, MSP_OP_SEND_TIME};
	unsigned char *send_buffers[2] = {test_send_pus, test_send_time};
	unsigned long send_sizes[2] = {sizeof(test_send_pus), sizeof(test_send_time)};
	const char *send_names[2] = {"PUS", "Time"};

	/* setup recv types */
	unsigned char recv_opcodes[3] = {MSP_OP_REQ_PAYLOAD, MSP_OP_REQ_HK, MSP_OP_REQ_PUS};
	const char *recv_names[3] = {"Payload", "Housekeeping", "PUS"};

	/* setup syscommand */
	unsigned char sys_opcodes[3] = {MSP_OP_ACTIVE, MSP_OP_SLEEP, MSP_OP_POWER_OFF};
	const char *sys_names[3] = {"ACTIVE", "SLEEP", "POWER_OFF"};

	if (transaction_type == 0) {
		static int send_type = 0;
		send_type = (send_type + 1) % sizeof(send_opcodes);

		test_send = send_buffers[send_type];
		test_send_size = send_sizes[send_type];
		send_data(send_opcodes[send_type], send_names[send_type]);
	} else if (transaction_type == 1) {
		static int recv_type = 0;
		recv_type = (recv_type + 1) % sizeof(recv_opcodes);

		receive_data(recv_opcodes[recv_type], recv_names[recv_type]);
	} else if (transaction_type == 2) {
		static int sys_type = 0;
		sys_type = (sys_type + 1) % sizeof(sys_opcodes);

		send_syscommand(sys_opcodes[sys_type], sys_names[sys_type]);
	}

	transaction_type = (transaction_type + 1) % 3;
	delay(3000);
}

/*
 * Sends data associated with the given opcode.
 *
 * Assumes that the send buffer and length is already configured before this is
 * called.
 */
static void send_data(unsigned char opcode, const char *name)
{
	struct msp_response r;

	Serial.print(F("[Starting Send "));
	Serial.print(name);
	Serial.println(F("]"));
	r = msp_start_transaction(&test_link, opcode, test_send_size);

	print_response(r, " - (OBC Send start)");
	while (msp_is_active(&test_link)) {
		r = process(&test_link);
		print_response(r, "");
		if (msp_error_count(&test_link) >= 5) {
			r = msp_abort_transaction(&test_link);
			Serial.println(F("Got 5 errors, aborting transaction."));
		}
	}
	Serial.println(F("[Transaction Ended]\n"));
}

/*
 * Receives data associated with the given opcode.
 */
static void receive_data(unsigned char opcode, const char *name)
{
	struct msp_response r;

	Serial.print(F("[Starting Request "));
	Serial.print(name);
	Serial.println(F("]"));
	r = msp_start_transaction(&test_link, opcode, 0);

	print_response(r, " - (OBC Request start)");
	while (msp_is_active(&test_link)) {
		r = process(&test_link);
		print_response(r, "");
		if (msp_error_count(&test_link) >= 5) {
			r = msp_abort_transaction(&test_link);
			Serial.println(F("Got 5 errors, aborting transaction."));
		}
	}
	if (r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL) {
		if (r.len < sizeof(test_recv))
			test_recv[r.len] = 0;
		else
			test_recv[sizeof(test_recv) - 1] = 0;

		/* Assume that we got a string. */
		Serial.println(F("----- RECEIVED DATA -----"));
		Serial.print(F("(Length: "));
		Serial.print(r.len, DEC);
		Serial.println(F(" bytes)"));
		Serial.println((char *) test_recv);
		Serial.println(F("-------------------------"));
	}
	Serial.println(F("[Transaction Ended]\n"));
}

/*
 * Sends a system command with the given opcode.
 */
static void send_syscommand(unsigned char opcode, const char *name)
{
	struct msp_response r;

	Serial.print(F("[Sending system command "));
	Serial.print(name);
	Serial.println(F("]"));
	r = msp_start_transaction(&test_link, opcode, 0);

	print_response(r, " - (OBC Send syscommand start)");
	while (msp_is_active(&test_link)) {
		r = process(&test_link);
		print_response(r, "");
		if (msp_error_count(&test_link) >= 5) {
			r = msp_abort_transaction(&test_link);
			Serial.println(F("Got 5 errors, aborting transaction."));
		}
	}
	Serial.println(F("[Transaction Ended]\n"));
}

/* Takes an action in an MSP transaction */
static struct msp_response process(msp_link_t *lnk)
{
	struct msp_response r;
	unsigned long len, offset;
	
	len = msp_next_data_length(lnk);
	offset = msp_next_data_offset(lnk);

	switch (msp_next_action(lnk)) {
	case MSP_LINK_ACTION_TX_HEADER:
		r = msp_send_header_frame(lnk);
		break;
	case MSP_LINK_ACTION_RX_HEADER:
		r = msp_recv_header_frame(lnk);
		break;
	case MSP_LINK_ACTION_TX_DATA:
		r = msp_send_data_frame(lnk, test_send + offset, len);
		break;
	case MSP_LINK_ACTION_RX_DATA:
		/* Make sure that we don't receive too much data */
		if ((len + offset) >= sizeof(test_recv))
			offset = 0;

		r = msp_recv_data_frame(lnk, test_recv + offset, &len);
		break;
	default:
		break;
	}

	return r;
}

/* A print function for printing out an MSP response */
static void print_response(struct msp_response r, const char *msg)
{
	switch (r.status) {
	case MSP_RESPONSE_BUSY:
		Serial.println(F("response: BUSY"));
		break;
	case MSP_RESPONSE_ERROR:
		Serial.print(F("response: ERROR: "));
		Serial.println(r.error_code, DEC);
		break;
	case MSP_RESPONSE_TRANSACTION_SUCCESSFUL:
		Serial.println(F("response: SUCCESSFUL"));
		break;
	case MSP_RESPONSE_TRANSACTION_ABORTED:
		Serial.println(F("response: ABORTED"));
		break;
	case MSP_RESPONSE_OK:
		Serial.println(F("response: OK"));
		break;
	}

	if (msg[0])
		Serial.println(msg);
}
